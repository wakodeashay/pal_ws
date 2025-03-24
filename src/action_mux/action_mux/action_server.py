import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from time import sleep
from custom_action.action import CustomGoal


class CustomActionServer(Node):
    def __init__(self):
        super().__init__("timed_action_server")

        self._server = ActionServer(
            self,
            CustomGoal,
            "goal_service",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Action server ready.")

    def goal_callback(self, goal_request: CustomGoal.Goal):
        '''Callback to accept/reject goal sent by client'''
        # Accept received goal simply!
        self.get_logger().info("Received goal request and Accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        '''callback to cancel goal sent by client'''
        # Accept camcellation request
        self.get_logger().info(f"Received cancel request for Goal: {self.command}")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        '''Callback to execute goal'''
        self.get_logger().info(f"Executing goal: {goal_handle.request.goal_command}")
        self.command = goal_handle.request.goal_command
        result = CustomGoal.Result()

        steps = 5.0
        time_period = 5.0

        # Execute goal in a simple loop with certain sleep duration
        for i in range(int(steps)):
            # Check if cancellation requested
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal was aborted.")
                goal_handle.canceled()
                break
            # No cancellation request -> continue execution
            else:
                feedback = CustomGoal.Feedback()
                feedback.percent_completion = (i + 1) * (100.0 / steps)
                self.get_logger().info(
                    f"Percentage Completetion: {feedback.percent_completion}"
                )
                goal_handle.publish_feedback(feedback)
                sleep(time_period / steps)

        # Check if the goal succeeded or was cancelled
        if i == steps - 1:
            goal_handle.succeed()
            self.get_logger().info("Goal succeeded.")
            result.success = True
        else:
            self.get_logger().info("Goal Cancellation Done.")
            result.success = False

        return result


def main():
    rclpy.init()
    node = CustomActionServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
