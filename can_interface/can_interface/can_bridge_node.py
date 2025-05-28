import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
import can
import csv
import os
from datetime import datetime

from can_interface.can_definitions import *


class CANBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')

        # Setup CAN bus (nur Empfang)
        self.bus_rx = can.interface.Bus(channel='can0', bustype='socketcan')

        # Setup CSV-Dateien
        self.can_log_file = 'can_log.csv'
        self.ros_log_file = 'ros_log.csv'
        self.init_logs()

        # ROS: Abo auf Hindernis-Topics
        self.hindernis_status = 0x00  # Bitmaske
        for i in range(1, 17):
            topic = f'/bool_topic_{i}'
            self.create_subscription(Bool, topic, self.create_hindernis_callback(i), 10)

        # ROS: Abo auf Fahrbefehl
        self.create_subscription(UInt8, '/fahrbefehl', self.fahrbefehl_callback, 10)

        # ROS: Publisher für freigegebene Fahrbefehle
        self.pub_freigabe = self.create_publisher(UInt8, '/freigegebener_fahrbefehl', 10)

        # Timer für CAN-Empfang (polling)
        self.create_timer(0.01, self.poll_can_bus)

    def init_logs(self):
        if not os.path.exists(self.can_log_file):
            with open(self.can_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'can_id', 'data'])

        if not os.path.exists(self.ros_log_file):
            with open(self.ros_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'topic', 'bit', 'value'])

    def poll_can_bus(self):
        msg = self.bus_rx.recv(timeout=0.01)
        if msg:
            ts = datetime.now().isoformat()
            can_id = hex(msg.arbitration_id)
            data = msg.data.hex()
            self.get_logger().info(f'CAN-Empfangen: ID={can_id}, Daten={data}')
            with open(self.can_log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([ts, can_id, data])

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
                writer = csv.writer(f)
                writer.writerow([ts, f'/bool_topic_{sektor}', bit, msg.data])

            self.get_logger().info(f'Hindernisstatus (ROS): 0b{self.hindernis_status:016b}')

        return callback

    def fahrbefehl_callback(self, msg):
        ts = datetime.now().isoformat()
        befehl = msg.data
        blockiert = False

        if befehl == FAHREN_VORWAERTS:
            if self.hindernis_status & sum(1 << i for i in [14, 15, 0, 1]):  # Sektor 15,16,1,2
                blockiert = True
        elif befehl == FAHREN_RUECKWAERTS:
            if self.hindernis_status & sum(1 << i for i in [6, 7, 8, 9]):  # Sektor 7,8,9,10
                blockiert = True

        if blockiert:
            self.get_logger().warn(f'Fahrbefehl blockiert: {befehl}')
        else:
            self.get_logger().info(f'Fahrbefehl weitergegeben: {befehl}')
            self.pub_freigabe.publish(msg)

        with open(self.ros_log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([ts, '/fahrbefehl', '-', befehl])


def main(args=None):
    rclpy.init(args=args)
    node = CANBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
