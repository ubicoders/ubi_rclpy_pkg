from rclpy.executors import MultiThreadedExecutor
from ubi_interface.srv import ExCalcOptPath
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

class CalcOptimalPath(Node):

    def __init__(self):
        super().__init__('optimal_path')
        self.get_logger().info("Starting service server...")
        self.callback_group = ReentrantCallbackGroup()

        self.optim_path_srv_server = self.create_service(
                ExCalcOptPath,
                'optim_path',
                self.srv_calc_optim,
                callback_group=self.callback_group)

    def srv_calc_optim(self, request, response):
        pt_end= request.target_position
        self.get_logger().info(f'Received target position: x={pt_end.x}, y={pt_end.y}, z={pt_end.z}')
        
        # Example path generation (simply returns a straight line towards the target)
        path = [Vector3(x=0.0, y=0.0, z=0.0),
                Vector3(x=pt_end.x/2.0, y=pt_end.y/2.0, z=pt_end.z/2.0),
                Vector3(x=pt_end.x, y=pt_end.y, z=pt_end.z)]
        response.path = path
        self.get_logger().info(f'returning target position: {path}')
        return response        


def main(args=None):
    rclpy.init(args=args)
    calculator = None
    try:
        calculator = CalcOptimalPath()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(calculator)
        try:
            executor.spin()
        except KeyboardInterrupt:
            calculator.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            if calculator:
                calculator.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()