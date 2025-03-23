import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from time import sleep
from custom_action.action import CustomGoal  

class CustomActionServer(Node):
    def __init__(self):
        super().__init__('timed_action_server')

        self._server = ActionServer(
            self,
            CustomGoal,
            'goal_service',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Action server ready.')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request and Accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(f'Received cancel request for Goal: {self.last_command}')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal: {goal_handle.request.goal_command}")
        self.last_command = goal_handle.request.goal_command

        steps = 50.0
        time_period = 5.0 
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Goal was cancelled.")
                goal_handle.canceled()
                break
            else:
                feedback = CustomGoal.Feedback()
                feedback.percent_completion = (i + 1) * (100.0 / steps)
                self.get_logger().info(f"Percentage Completetion: {feedback.percent_completion}")
                goal_handle.publish_feedback(feedback)
                sleep(time_period/steps)
        
        if i == steps - 1:
            goal_handle.succeed()
            self.get_logger().info("Goal succeeded.")
            return CustomGoal.Result(success=True)
        else:
            self.get_logger().info("Goal Aborted.")
            result = CustomGoal.Result()
            result.success = False
            return result


def main():
    rclpy.init()
    node = CustomActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
