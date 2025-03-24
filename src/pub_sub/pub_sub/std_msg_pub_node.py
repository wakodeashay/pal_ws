import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class StdMsgPublisher(Node):
    def __init__(self):
        super().__init__('std_msgs_publisher')
        self.geometry_msgs_publisher_ = self.create_publisher(Bool, '/common_topic', 10)
        self.pub_timer_ = self.create_timer(2.0, self.publish_message)
        self.count_ = 0

    def publish_message(self):
        '''Timer Callback'''
        msg = Bool()
        
        # Change between True and False between every 10 iterations
        mod_count_ = int(self.count_/10)

        if mod_count_%2 == 0:
            msg.data = False
        else:
            msg.data = True

        self.geometry_msgs_publisher_.publish(msg)
        self.get_logger().info(f"Published Bool data: {msg.data}")

        self.count_ = self.count_ + 1

def main():
    rclpy.init()
    node = StdMsgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()