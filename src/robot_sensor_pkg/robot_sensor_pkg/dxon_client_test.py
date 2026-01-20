#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class dxonClientTest(Node):
    def __init__(self):
        super().__init__("dxon_client_test")
        # Create client
        self.client = self.create_client(Trigger, 'connect_dxon')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # Prepare request
        self.request = Trigger.Request()
        # Send request
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    node = dxonClientTest()

    # Spin until response is received
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
                if response.success:
                    node.get_logger().info(f"Service call succeeded: {response.message}")
                else:
                    node.get_logger().warn(f"Service call failed: {response.message}")
            except Exception as e:
                node.get_logger().error(f"Service call failed with exception: {e}")
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
