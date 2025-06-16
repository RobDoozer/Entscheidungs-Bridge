#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import csv
import os
from datetime import datetime

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

from can_interface.can_definitions import *

class CANBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')

        self.logger = self.get_logger()

        self.bus_rx = None
        if CAN_AVAILABLE:
            try:
                self.bus_rx = can.interface.Bus(channel='can0', bustype='socketcan')
                self.logger.info("CAN-Schnittstelle erfolgreich initialisiert.")
            except Exception as e:
                self.logger.warn(f'CAN konnte nicht geöffnet werden: {e}')

        self.can_log_file = 'can_log.csv'
        self.ros_log_file = 'ros_log.csv'
        self.init_logs()

        self.hindernis_status = 0x00
        for i in range(1, 17):
            topic = f'/bool_topic_{i}'
            self.create_subscription(Bool, topic, self.create_hindernis_callback(i), 10)

        self.create_subscription(Float32MultiArray, '/controller_commands', self.controller_callback, 10)
        self.pub_freigabe = self.create_publisher(Float32MultiArray, '/freigegebener_fahrbefehl', 10)

        self.last_forward = None
        self.last_turn = None

        if self.bus_rx:
            self.create_timer(0.01, self.poll_can_bus)

    def init_logs(self):
        if not os.path.exists(self.can_log_file):
            with open(self.can_log_file, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'can_id', 'data'])
        if not os.path.exists(self.ros_log_file):
            with open(self.ros_log_file, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp', 'topic', 'bit', 'value'])

    def poll_can_bus(self):
        msg = self.bus_rx.recv(timeout=0.01)
        if msg:
            ts = datetime.now().isoformat()
            can_id = hex(msg.arbitration_id)
            data = msg.data.hex()
            self.logger.info(f'CAN-Empfangen: ID={can_id}, Daten={data}')
            with open(self.can_log_file, 'a', newline='') as f:
                csv.writer(f).writerow([ts, can_id, data])

    def create_hindernis_callback(self, sektor):
        def callback(msg):
            bit = sektor - 1
            mask = 1 << bit
            ts = datetime.now().isoformat()

            if msg.data:
                self.hindernis_status |= mask
            else:
                self.hindernis_status &= ~mask

            with open(self.ros_log_file, 'a', newline='') as f:
                csv.writer(f).writerow([ts, f'/bool_topic_{sektor}', bit, msg.data])

            self.logger.info(f'Hindernisstatus (ROS): 0b{self.hindernis_status:016b}')
        return callback

    def controller_callback(self, msg):
        ts = datetime.now().isoformat()
        if len(msg.data) < 2:
            self.logger.warn("Ungültiger Controller-Input (weniger als 2 Werte)")
            return

        forward = msg.data[0]
        turn = msg.data[1]
        original_forward = forward
        original_turn = turn

        translation_blocked = False
        rotation_blocked = False

        if forward > 0 and self.hindernis_status & sum(1 << i for i in [14, 15, 0, 1]):
            translation_blocked = True
        elif forward < 0 and self.hindernis_status & sum(1 << i for i in [6, 7, 8, 9]):
            translation_blocked = True

        if turn > 0 and self.hindernis_status & sum(1 << i for i in [3, 4, 5, 6]):
            rotation_blocked = True
        elif turn < 0 and self.hindernis_status & sum(1 << i for i in [11, 12, 13, 14]):
            rotation_blocked = True

        if translation_blocked:
            forward = 0.0
        if rotation_blocked:
            turn = 0.0

        def has_changed(new_fwd, new_turn, old_fwd, old_turn):
            return (round(new_fwd, 3) != round(old_fwd, 3)) or (round(new_turn, 3) != round(old_turn, 3))

        if self.last_forward is not None and self.last_turn is not None:
            if not has_changed(forward, turn, self.last_forward, self.last_turn):
                self.logger.debug('Fahrbefehl unverändert (auf 3 Dezimalstellen), nicht erneut gesendet.')
                return

        # Log-Ausgabe nur wenn es eine Änderung gab:
        if translation_blocked or rotation_blocked:
            self.logger.warn(f'Fahrbefehl blockiert: [{original_forward:.3f}, {original_turn:.3f}] → [{forward:.3f}, {turn:.3f}]')
        else:
            self.logger.info(f'Fahrbefehl freigegeben: [{forward:.3f}, {turn:.3f}]')

        self.last_forward = forward
        self.last_turn = turn

        msg_out = Float32MultiArray()
        msg_out.data = [forward, turn]
        self.pub_freigabe.publish(msg_out)

        with open(self.ros_log_file, 'a', newline='') as f:
            csv.writer(f).writerow([ts, '/controller_commands', '-', f'{forward:.3f},{turn:.3f}'])

def main(args=None):
    rclpy.init(args=args)
    node = CANBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

