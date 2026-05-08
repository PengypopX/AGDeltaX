#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
import tf2_ros
import numpy as np
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time




# Calibrated intrinsics at 1280x960
FX = 867.120437
FY = 874.307409
CX = 626.253675
CY = 588.498914




class YoloPixelToMMv2(Node):
    def __init__(self):
        super().__init__('yolo_pixel_to_mm_v2')


        self.declare_parameter(
            'extrinsics_path',
            '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml'
        )
        extrinsics_path = self.get_parameter('extrinsics_path').value


        with open(extrinsics_path, 'r') as f:
            extr = yaml.safe_load(f)
        self.camera_height_mm = extr['translation']['z'] * 1000.0
        self.get_logger().info(f'Camera height Z: {self.camera_height_mm:.1f}mm')


        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detection_callback,
            10
        )
        self.get_logger().info('yolo_pixel_to_mm_v2 started (pinhole model)')


    def pixel_to_world_mm(self, u, v):
        """
        Pinhole camera model:
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
        Assumes camera is roughly perpendicular to ground plane.
        Z is camera height above working surface in mm.
        """
        Z = self.camera_height_mm
        X_mm = (u - CX) * Z / FX
        Y_mm = (v - CY) * Z / FY
        Y_mm = max(Y_mm, -200.0)  # Safety floor — protect camera
        return X_mm, Y_mm, Z


    def get_robot_to_weed_vector(self, weed_id):
        try:
            weed_tf = self.tf_buffer.lookup_transform(
                'robot_home',
                f'weed_{weed_id}',
                Time()
            )
            dx = weed_tf.transform.translation.x * 1000.0
            dy = weed_tf.transform.translation.y * 1000.0
            gcode = f'G1 X{int(dx)} Y{int(dy)} Z-800'
            self.get_logger().info(
                f'Weed {weed_id} -> robot_home: X={dx:.1f}mm Y={dy:.1f}mm | {gcode}'
            )
            return dx, dy, gcode
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None, None, None


    def detection_callback(self, msg):
        for detection in msg.detections:
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y


            X_mm, Y_mm, Z_mm = self.pixel_to_world_mm(u, v)


            self.get_logger().info(
                f'[V2] ID: {detection.id} | Class: {detection.class_name} | '
                f'Score: {detection.score:.2f} | '
                f'Pixel: ({u:.1f}, {v:.1f}) | '
                f'World: ({X_mm:.1f}mm, {Y_mm:.1f}mm, {Z_mm:.1f}mm)'
            )


            self.publish_weed_tf(detection.id, X_mm, Y_mm, Z_mm, msg.header.stamp)
            self.get_robot_to_weed_vector(detection.id)


    def publish_weed_tf(self, weed_id, X_mm, Y_mm, Z_mm, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'camera'
        tf_msg.child_frame_id = f'weed_{weed_id}'


        tf_msg.transform.translation.x = X_mm / 1000.0
        tf_msg.transform.translation.y = Y_mm / 1000.0
        tf_msg.transform.translation.z = Z_mm / 1000.0


        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0


        self.tf_broadcaster.sendTransform(tf_msg)




def main(args=None):
    rclpy.init(args=args)
    node = YoloPixelToMMv2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()




