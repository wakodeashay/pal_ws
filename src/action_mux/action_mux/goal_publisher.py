import rclpy
import random
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int16

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Declare and get initial publish rate
        self.declare_parameter('publish_rate', 0.1)  
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Int16, '/goal_topic', 10)

        # Create timer with initial rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Check for param updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f"Starting publisher at {self.publish_rate:.2f} Hz")

    def publish_callback(self):
        msg = Int16()
        msg.data = random.randint(0,9)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}', at rate {self.publish_rate:.2f} Hz")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'publish_rate' and param.type_ == param.Type.DOUBLE:
                new_rate = param.value
                if new_rate > 0.0:
                    self.publish_rate = new_rate
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)
                    self.get_logger().info(f"Updated publish rate to {self.publish_rate:.2f} Hz")
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().warn("Publish rate must be > 0.0 Ignoring.")
                    return SetParametersResult(successful=False)
def main():
    rclpy.init()
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
