import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

class SimulatedController(Node):
    def __init__(self):
        super().__init__('simulated_controller')
        self.publisher = self.create_publisher(UInt8, '/fahrbefehl', 10)
        self.timer = self.create_timer(2.0, self.send_command)

    def send_command(self):
        msg = UInt8()
        msg.data = 1  # z. B. FAHREN_VORWAERTS
        self.publisher.publish(msg)
        self.get_logger().info(f'Sende Fahrbefehl: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

