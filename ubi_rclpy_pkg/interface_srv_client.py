import rclpy
from rclpy.node import Node
from ubi_interface.srv import ExCalcOptPath
from geometry_msgs.msg import Vector3

class ExCalcOptPathClientNode(Node):
    def __init__(self):
        super().__init__('ex_calc_opt_path_client')
        self.client = self.create_client(ExCalcOptPath, '/optim_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/optim_path service not available, waiting again...')
        self.req = ExCalcOptPath.Request()

    def send_request(self):
        self.req.target_position = Vector3(x=50.0, y=0.0, z=0.0)
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    ex_calc_opt_path_client = ExCalcOptPathClientNode()
    ex_calc_opt_path_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(ex_calc_opt_path_client)
        if ex_calc_opt_path_client.future.done():
            try:
                response = ex_calc_opt_path_client.future.result()
                ex_calc_opt_path_client.get_logger().info(f'Received response: {response}')
            except Exception as e:
                ex_calc_opt_path_client.get_logger().error(f'Service call failed %r' % (e,))
            break

    ex_calc_opt_path_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()