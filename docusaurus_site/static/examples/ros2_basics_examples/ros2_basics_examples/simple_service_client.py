import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run ros2_basics_examples simple_service_client <int1> <int2>")
        return

    simple_service_client = SimpleServiceClient()
    simple_service_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(simple_service_client)
        if simple_service_client.future.done():
            try:
                response = simple_service_client.future.result()
            except Exception as e:
                simple_service_client.get_logger().info(
                    f'Service call failed {e}')
            else:
                simple_service_client.get_logger().info(
                    f'Result of add_two_ints: for {simple_service_client.req.a} + {simple_service_client.req.b} = {response.sum}')
            break

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
