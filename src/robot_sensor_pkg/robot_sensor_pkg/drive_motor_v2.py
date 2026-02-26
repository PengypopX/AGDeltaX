#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import tf2_ros
from rclpy.time import Time
import deltax


class drive_motor_v2(Node):
    def __init__(self):
        # initialization
        super().__init__("drive_motor_v2")

        # variable section
        self.x = 0
        self.y = 0
        self.z = 0
        self.last_x = 1.5
        self.last_y = .53
        self.last_z = -740
        self.F = 100
        # Prior homing values X = 1.5 Y = .53 Z = -740

        self.get_logger().info("dxon node has been started.")

        # TF listener for weed positions
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Creating deltax node here
        self.dx = deltax.DeltaX(port="/dev/ttyUSB0", model=deltax.DeltaX.DeltaX_S)

        self.dx.connect()
        self.setSpeed(self.F)
        # self.drive_z(-800)
    # IDEAL Vector: X100 Y100 Z100 F100
    def setSpeed(self, pass_f):
        self.dx.send(f"G1 F{pass_f}")

    def drive_x(self, pass_x):
        self.dx.send(f"G1 X{pass_x}")
        self.last_x = pass_x

    def drive_y(self, pass_y):
        self.dx.send(f"G1 Y{pass_y}")
        self.last_y = pass_y

    def drive_z(self, pass_z):
        self.dx.send(f"G1 Z{pass_z}")
        self.last_z = pass_z

    def drive_x_and_y(self, pass_x, pass_y):
        self.dx.send(f"G1 X{pass_x} Y{pass_y}")

    def drive_thru(self, pass_x, pass_y, pass_z):
        self.drive_x_and_y(pass_x, pass_y)
        self.drive_z(pass_z)
        self.dx.send("G28")  # home
        self.get_logger().info("drive_thru_executed")

    def reset_pos(self):
        self.dx.homing()

    # --- Weed targeting ---
    def get_weed_position(self, weed_id):
        """Look up weed position from TF tree in robot arm frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'robot_arm_base',    # target frame - robot's coordinate frame
                f'weed_{weed_id}',   # source frame - published by yolo_pixel_to_mm
                Time()
            )
            x_mm = tf.transform.translation.x * 1000.0
            y_mm = tf.transform.translation.y * 1000.0
            self.get_logger().info(f'Weed {weed_id} position: X={x_mm:.1f}mm Y={y_mm:.1f}mm')
            return x_mm, y_mm

        except Exception as e:
            self.get_logger().warn(f'Could not get weed {weed_id} position: {e}')
            return None, None

    X_OFFSET = -98.3
    Y_OFFSET = -24.9
    def move_to_weed(self, weed_id):
        """Move robot arm to weed position."""
        x_mm, y_mm = self.get_weed_position(weed_id)
        if x_mm is not None:
            x_mm += self.X_OFFSET
            y_mm += self.Y_OFFSET
            self.get_logger().info(f'Moving to weed {weed_id}: X={x_mm:.1f} Y={y_mm:.1f}')
            self.drive_x_and_y(x_mm, y_mm)
        else:
            self.get_logger().warn(f'Weed {weed_id} not found in TF tree')

    def connect_dxon_callback(self, request, response):
        self.dx.connect()
        if self.dx.is_connected():
            self.get_logger().info("Connection success [ROS2]")
            response.success = True
            response.message = "DeltaX connected."
        else:
            self.get_logger().error("Failed to connect [ROS2]")
            response.success = False
            response.message = "DeltaX is not connected. Ensure all connections are secure."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = drive_motor_v2()
    # Let TF buffer populate
    for _ in range(8):
        rclpy.spin_once(node, timeout_sec=0.5)
    
    node.move_to_weed(1)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()