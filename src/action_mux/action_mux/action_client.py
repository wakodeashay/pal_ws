import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.subscription import Subscription

from std_msgs.msg import Int16
from action_msgs.msg import GoalStatus
from custom_action.action import CustomGoal


class CustomActionClient(Node):
    def __init__(self):
        super().__init__('timed_action_client')
        
        # Topic name to subscribe goal
        goal_topic_name = '/goal_topic'

        self.action_client_ = ActionClient(self, CustomGoal, 'goal_service')

        self.goal_subscription = self.create_subscription(
            Int16,
            goal_topic_name,
            self.goal_callback,
            QoSProfile(depth=10)
        )

        self.count_ = 0
        self.new_goal_ = 0
        self.prev_goal = 0
        self.get_logger().info('Action client ready. Waiting for Server!')

    def goal_callback(self, msg):
        '''Goal Callback'''
        self.goal = msg.data
        self.get_logger().info(f"Received new goal command: {self.goal}")
        self.new_goal_ = self.goal
        self.action_client_.wait_for_server()

        # Cancel existing goal incase of new except for the first time
        if self.count_ !=0:
            # Abort previous goal if still running
            if self.goal_handle_.status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().warn("Cancelling previous goal..." + str(self.prev_goal_))
                self.goal_handle_.cancel_goal_async()

        self.get_logger().info("Sending new goal..." + str(self.new_goal_))
        self.send_goal(self.goal)
        self.prev_goal_ = self.new_goal_

    def send_goal(self, goal_value):
        '''Function to send goal action'''
        self.count_ = self.count_ + 1 
        goal = CustomGoal.Goal()
        goal.goal_command = goal_value

        # Send goal
        self.action_client_. \
            send_goal_async(goal).\
            add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''Callback to check if the goal is accepted or rejected'''
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.\
            get_result_async().\
            add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        '''Callback to check if the goal succeded or cancelled'''
        status = future.result().status 
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded !")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal Canceled !")

        self.get_logger().info("Result: " + str(result.success))


def main():
    rclpy.init()
    node = CustomActionClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
