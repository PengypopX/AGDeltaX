#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.time import Time
from dx_interfaces.srv import SendString

class RobotPoll(Node):
    def __init__(self):
        super().__init__('robot_poll')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client = self.create_client(SendString, 'raw_gcode')

        # Check for weeds every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.is_moving = False

        self.get_logger().info('robot_poll started')

    def timer_callback(self):
        if self.is_moving:
            return

        for weed_id in range(1, 11):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'robot_home',
                    f'weed_{weed_id}',
                    Time()
                )
                x_mm = int(tf.transform.translation.x * 1000.0)
                y_mm = int(tf.transform.translation.y * 1000.0)

                gcode = f'G1 X{x_mm} Y{y_mm} Z-800'
                self.get_logger().info(f'Found weed {weed_id}: {gcode}')

                if self.client.wait_for_service(timeout_sec=1.0):
                    self.is_moving = True
                    request = SendString.Request()
                    request.data = gcode
                    future = self.client.call_async(request)
                    future.add_done_callback(self.send_ts_callback)
                return

            except Exception:
                pass

    def send_ts_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Move complete')
        else:
                self.get_logger().warn(f'Move failed')
        self.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()