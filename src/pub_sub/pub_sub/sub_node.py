import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rosidl_runtime_py.utilities import get_message

class GeneralSubscriber(Node):
    def __init__(self):
        super().__init__('general_subscriber')

        topic_name = '/common_topic'
        topics = self.get_topic_names_and_types()

        for topic, msg_type in topics:
            self.msg_type = get_message(msg_type[0])
            if topic_name == topic:
                self.get_logger().info(f"Topic name {topic_name} available!")
                self.get_logger().info(f"Subscribing to topic: {topic_name} with type: {msg_type}")

                self.subscription = self.create_subscription(
                    self.msg_type,
                    topic_name,
                    self.subscriber_callback,
                    QoSProfile(depth=10)
                )

    def subscriber_callback(self, msg):
        self.get_logger().info(f"Received message of type {self.msg_type}: {msg}")

def main():
    rclpy.init()
    node = GeneralSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
