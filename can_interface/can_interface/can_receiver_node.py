import rclpy
from rclpy.node import Node
import can
from datetime import datetime

class CANReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.timer = self.create_timer(0.01, self.read_can)

    def read_can(self):
        msg = self.bus.recv(timeout=0.01)
        if msg:
            data_list = list(msg.data)
            hex_data = ' '.join(f'{b:02X}' for b in data_list)
            self.get_logger().info(f'Empfangen: ID=0x{msg.arbitration_id:X}, Daten={hex_data}')

def main(args=None):
    rclpy.init(args=args)
    node = CANReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
