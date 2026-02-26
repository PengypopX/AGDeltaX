#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
import tf2_ros
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped

class YoloPixelToMM(Node):
    def __init__(self):
        super().__init__('yolo_pixel_to_mm')

        # --- Load calibration files ---
        self.declare_parameter('intrinsics_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/intrinsics.yaml')
        self.declare_parameter('extrinsics_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics_inv.yaml')

        intrinsics_path = self.get_parameter('intrinsics_path').value
        extrinsics_path = self.get_parameter('extrinsics_path').value

        self.H = self.load_calibration(intrinsics_path, extrinsics_path)

        # --- TF broadcaster ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Subscribe to YOLO tracking ---
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detection_callback,
            10
        )
        self.get_logger().info('pixel_to_mm node started')

    def load_calibration(self, intrinsics_path, extrinsics_path):
        # --- Load intrinsics ---
        with open(intrinsics_path, 'r') as f:
            intr = yaml.safe_load(f)
        K = np.array(intr['camera_matrix']['data']).reshape(3, 3)

        # --- Load extrinsics ---
        with open(extrinsics_path, 'r') as f:
            extr = yaml.safe_load(f)
        t = np.array([
            extr['translation']['x'],
            extr['translation']['y'],
            extr['translation']['z']
        ])
        q = [
            extr['rotation']['x'],
            extr['rotation']['y'],
            extr['rotation']['z'],
            extr['rotation']['w']
        ]
        R = Rotation.from_quat(q).as_matrix()

        # --- Compute homography for ground plane (Z=0) ---
        H = K @ np.column_stack([R[:, 0], R[:, 1], t])
        self.get_logger().info('Calibration loaded and homography computed')
        return H

    def pixel_to_world_mm(self, u, v):
        """Convert pixel (u, v) to real-world (X, Y) in millimeters on ground plane."""
        pixel = np.array([u, v, 1.0])
        world = np.linalg.solve(self.H, pixel)
        world /= world[2]
        X_mm = world[0] * 1000.0
        Y_mm = world[1] * 1000.0
        return X_mm, Y_mm

    def detection_callback(self, msg):
        for detection in msg.detections:
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y
            X_mm, Y_mm = self.pixel_to_world_mm(u, v)

            self.get_logger().info(
                f'ID: {detection.id} | Class: {detection.class_name} | '
                f'Score: {detection.score:.2f} | '
                f'Pixel: ({u:.1f}, {v:.1f}) | '
                f'World: ({X_mm:.1f}mm, {Y_mm:.1f}mm)'
            )

            self.publish_weed_tf(detection.id, X_mm, Y_mm, msg.header.stamp)

    def publish_weed_tf(self, weed_id, X_mm, Y_mm, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'floor'
        tf_msg.child_frame_id = f'weed_{weed_id}'

        tf_msg.transform.translation.x = X_mm / 1000.0
        tf_msg.transform.translation.y = Y_mm / 1000.0
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloPixelToMM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()