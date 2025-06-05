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

        # Optional: CAN
        self.bus_rx = None
        if CAN_AVAILABLE:
            try:
                self.bus_rx = can.interface.Bus(channel='can0', bustype='socketcan')
                self.logger.info("CAN-Schnittstelle erfolgreich initialisiert.")
            except Exception as e:
                self.get_logger().warn(f'CAN konnte nicht geöffnet werden: {e}')

        # CSV-Logs
        self.can_log_file = 'can_log.csv'
        self.ros_log_file = 'ros_log.csv'
        self.init_logs()

        # Hindernisstatus
        self.hindernis_status = 0x00
        for i in range(1, 17):
            topic = f'/bool_topic_{i}'
            self.create_subscription(Bool, topic, self.create_hindernis_callback(i), 10)

        # Neue ROS-Fahrbefehl-Quelle: Float32MultiArray
        self.create_subscription(Float32MultiArray, '/controller_commands', self.controller_callback, 10)

        # Ziel: Nur wenn freigegeben → weiterleiten
        self.pub_freigabe = self.create_publisher(Float32MultiArray, '/freigegebener_fahrbefehl', 10)

        # Optional: CAN polling
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

        forward_backward = msg.data[0]
        drehen = msg.data[1]
        blockiert = False

        if forward_backward > 0:
            if self.hindernis_status & sum(1 << i for i in [14, 15, 0, 1]):
                blockiert = True
        elif forward_backward < 0:
            if self.hindernis_status & sum(1 << i for i in [6, 7, 8, 9]):
                blockiert = True

        if blockiert:
            self.logger.warn(f'Fahrbefehl blockiert: [{forward_backward:.2f}, {drehen:.2f}]')
        else:
            self.logger.info(f'Fahrbefehl freigegeben: [{forward_backward:.2f}, {drehen:.2f}]')
            new_msg = Float32MultiArray()
            new_msg.data = [forward_backward, drehen]
            self.pub_freigabe.publish(new_msg)

        with open(self.ros_log_file, 'a', newline='') as f:
            csv.writer(f).writerow([ts, '/controller_commands', '-', f'{forward_backward:.2f},{drehen:.2f}'])


def main(args=None):
    rclpy.init(args=args)
    node = CANBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

