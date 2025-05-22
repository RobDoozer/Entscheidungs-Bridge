import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import UInt8MultiArray

class CANSender(Node):
    def __init__(self):
        super().__init__('can_sender')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'can_tx',
            self.send_can,
            10
        )

    def send_can(self, msg):
        try:
            can_msg = can.Message(arbitration_id=0x123, data=msg.data, is_extended_id=False)
            self.bus.send(can_msg)
            self.get_logger().info(f'Sende CAN: {msg.data}')
        except can.CanError as e:
            self.get_logger().error(f'Fehler beim Senden: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CANSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
