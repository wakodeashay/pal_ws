import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class SensorMsgPublisher(Node):
    def __init__(self):
        super().__init__('sensor_msgs_publisher')
        self.sensor_msgs_publisher_ = self.create_publisher(Imu, '/common_topic', 10)
        self.pub_timer_ = self.create_timer(2.0, self.publish_message)

    def publish_message(self):
        '''Timer Callback'''
        msg = Imu()
        data = Quaternion()
        data.x = 0.0
        data.y = 0.0
        data.z = 0.0
        data.w = 1.0

        msg.orientation = data
        self.sensor_msgs_publisher_.publish(msg)
        self.get_logger().info(f"Published Imu data; with Quaternion: {data}")

def main():
    rclpy.init()
    node = SensorMsgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()