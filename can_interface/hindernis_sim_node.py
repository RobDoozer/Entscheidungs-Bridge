import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random

class MultiBoolPublisher(Node):
    def __init__(self):
        super().__init__('multi_bool_publisher')

        self.bool_publishers = []
        self.topic_names = [f'/bool_topic_{i+1}' for i in range(16)]

        for topic in self.topic_names:
            publisher = self.create_publisher(Bool, topic, 10)
            self.bool_publishers.append(publisher)
            self.get_logger().info(f'Publisher für {topic} erstellt')

        self.timer = self.create_timer(0.5, self.publish_bools)

    def publish_bools(self):
        for i, publisher in enumerate(self.bool_publishers):
            msg = Bool()
            msg.data = random.choice([True, False])
            publisher.publish(msg)
            self.get_logger().info(f'{self.topic_names[i]} → {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiBoolPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
