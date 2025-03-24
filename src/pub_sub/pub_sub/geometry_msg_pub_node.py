import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class GeometryMsgPublisher(Node):
    def __init__(self):
        super().__init__('geometry_msgs_publisher')
        self.geometry_msgs_publisher_ = self.create_publisher(Vector3, '/common_topic', 10)
        self.pub_timer_ = self.create_timer(2.0, self.publish_message)

    def publish_message(self):
        '''Timer callback'''
        msg = Vector3()
        msg.x = 8.0
        msg.y = 9.0
        msg.z = 10.0

        self.geometry_msgs_publisher_.publish(msg)
        self.get_logger().info(f"Published Vector data: {msg}")

def main():
    rclpy.init()
    node = GeometryMsgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()