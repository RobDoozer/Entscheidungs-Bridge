import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import random

class ControllerInputNode(Node):
    def __init__(self):
        super().__init__('controller_input_node')
        self.publisher_ = self.create_publisher(UInt8, 'fahrbefehl', 10)
        timer_period = 1.0  # alle 1 Sekunde
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('ControllerInputNode gestartet')

    def timer_callback(self):
        msg = UInt8()
        msg.data = random.choice([0, 1, 2])  # 0 = nix, 1 = vorwärts, 2 = rückwärts
        self.publisher_.publish(msg)
        befehle = {0: 'kein Befehl', 1: 'vorwärts', 2: 'rückwärts'}
        self.get_logger().info(f'Neuer Fahrbefehl: {msg.data} ({befehle[msg.data]})')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
