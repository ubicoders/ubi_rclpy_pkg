import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ubi_interface.action import ExTakeOff

class AltitudeActionClient(Node):

    def __init__(self):
        super().__init__('altitude_action_client')
        self._action_client = ActionClient(self, ExTakeOff, 'altitude_action')

    def send_goal(self, altitude):
        goal_msg = ExTakeOff.Goal()
        goal_msg.altitude = altitude

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final Altitude: {result.current_alt:.2f}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Error Altitude {feedback.error_alt:.2f}')

def main(args=None):
    rclpy.init(args=args)
    action_client = AltitudeActionClient()
    action_client.send_goal(10.0)  # Example altitude goal

    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()