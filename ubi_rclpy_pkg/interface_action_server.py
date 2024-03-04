import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ubi_interface.action import ExTakeOff
from std_msgs.msg import Float32

class AltitudeActionServer(Node):

    def __init__(self):
        super().__init__('altitude_action_server')
        self._action_server = ActionServer(
            self,
            ExTakeOff,
            'altitude_action',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = ExTakeOff.Feedback()

        # Simulate altitude change
        current_altitude = 0.0  # Starting altitude
        target_altitude = goal_handle.request.altitude
        error_margin = 0.1  # Just for simulation purposes

        while abs(target_altitude - current_altitude) > error_margin:
            # Simulate some altitude change
            current_altitude += (target_altitude - current_altitude) / 2
            feedback_msg.error_alt = target_altitude - current_altitude
            self.get_logger().info(f'Feedback: Error Altitude {feedback_msg.error_alt}')
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()

        result = ExTakeOff.Result()
        result.current_alt = current_altitude
        return result

def main(args=None):
    rclpy.init(args=args)
    altitude_action_server = AltitudeActionServer()

    rclpy.spin(altitude_action_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()