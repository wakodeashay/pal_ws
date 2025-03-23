import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from std_msgs.msg import Int16
from action_msgs.msg import GoalStatus
from rclpy.subscription import Subscription
from custom_action.action import CustomGoal


class CustomActionClient(Node):
    def __init__(self):
        super().__init__('action_client')

        goal_topic_name = '/goal_topic'

        self.action_client_ = ActionClient(self, CustomGoal, 'goal_service')
        self.goal_handle_ = None

        self.goal_subscription = self.create_subscription(
            Int16,
            goal_topic_name,
            self.goal_callback,
            QoSProfile(depth=10)
        )

        self.get_logger().info('Action client ready.')

    def goal_callback(self, msg):
        self.goal = msg.data
        self.get_logger().info(f"Received new goal command: {self.goal}")

        # Cancel previous goal if still running
        if self.goal_handle_ and self.goal_handle_.status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().warn("Cancelling previous goal...")
            # self.goal_handle_.cancel_goal_async()
            cancel_future = self.goal_handle_.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancel_done)

        self.get_logger().info("Sending new goal...")
        # goal_msg = CustomGoal.goal_command()
        goal_msg = CustomGoal.Goal()
        goal_msg.goal_command = self.goal
        future = self.action_client_.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def on_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Previous goal successfully canceled.")
        else:
            self.get_logger().warn("Previous goal could not be canceled or was already done.")

    def goal_response_callback(self, future):
        self.goal_handle_ = future.result()
        self.goal_handle_.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Goal completed successfully.")
        else:
            self.get_logger().warn("Goal failed or was aborted.")


def main():
    rclpy.init()
    node = CustomActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
