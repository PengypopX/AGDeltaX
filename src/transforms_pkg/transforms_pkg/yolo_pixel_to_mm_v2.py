#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
import tf2_ros
import numpy as np
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

KNOWN_SIZES_MM = {
    'penny': 19.05,
    'dime': 17.91,
    'nickel': 21.21,
    'quarter': 24.26,
    'weed': 50.0
}

class YoloPixelToMMv2(Node):
    def __init__(self):
        super().__init__('yolo_pixel_to_mm_v2')

        self.declare_parameter('homography_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/homography.yaml')
        self.declare_parameter('extrinsics_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml')

        homography_path = self.get_parameter('homography_path').value
        extrinsics_path = self.get_parameter('extrinsics_path').value

        # Load camera height from extrinsics
        with open(extrinsics_path, 'r') as f:
            extr = yaml.safe_load(f)
        self.camera_height_mm = extr['translation']['z'] * 1000.0
        self.get_logger().info(f'Camera height: {self.camera_height_mm:.1f}mm')

        # Load homography from yaml
        self.cam2world = self.load_homography(homography_path)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detection_callback,
            10
        )
        self.get_logger().info('pixel_to_mm_v2 node started')

    def load_homography(self, homography_path):
        with open(homography_path, 'r') as f:
            data = yaml.safe_load(f)
        cam2world = np.array(data['cam2world']['data']).reshape(3, 3)
        self.get_logger().info(f'Homography loaded:\n{cam2world}')
        return cam2world

    def get_scale_factor(self, class_name, bbox_size_x, bbox_size_y):
        real_size_mm = KNOWN_SIZES_MM.get(class_name.lower(), None)
        if real_size_mm is None:
            self.get_logger().warn(f'Unknown class: {class_name}, using fallback')
            real_size_mm = 20.0
        avg_pixel_size = (bbox_size_x + bbox_size_y) / 2.0
        scale_factor = real_size_mm / avg_pixel_size
        return scale_factor

    def pixel_to_world_mm(self, u, v, scale_factor):
        # Scale to homography resolution then center
        u_scaled = u * (640.0 / 1280.0)
        v_scaled = v * (480.0 / 960.0)
        u_centered = u_scaled - 320.0
        v_centered = 240.0 - v_scaled

        import cv2
        pixel = np.array([[u_centered, v_centered]], dtype=np.float32).reshape(-1, 1, 2)
        world = cv2.perspectiveTransform(pixel, self.cam2world)
        X_mm = world[0][0][0] * 1000.0
        Y_mm = world[0][0][1] * 1000.0
        Y_mm = max(Y_mm, -200.0)
        Z_mm = self.camera_height_mm
        return X_mm, Y_mm, Z_mm

    def get_robot_to_weed_vector(self, weed_id):
        try:
            weed_tf = self.tf_buffer.lookup_transform(
                'robot_home',
                f'weed_v2_{weed_id}',
                Time()
            )
            dx = weed_tf.transform.translation.x * 1000.0
            dy = weed_tf.transform.translation.y * 1000.0
            gcode = f'G1 X{int(dx)} Y{int(dy)} Z-800'
            self.get_logger().info(f'Weed {weed_id} vector: X={dx:.1f}mm Y={dy:.1f}mm | {gcode}')
            return dx, dy, gcode
        except Exception as e:
            self.get_logger().warn(f'Could not compute vector: {e}')
            return None, None, None

    def detection_callback(self, msg):
        for detection in msg.detections:
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y
            bbox_size_x = detection.bbox.size.x
            bbox_size_y = detection.bbox.size.y

            scale_factor = self.get_scale_factor(
                detection.class_name,
                bbox_size_x,
                bbox_size_y
            )

            X_mm, Y_mm, Z_mm = self.pixel_to_world_mm(u, v, scale_factor)

            self.get_logger().info(
                f'[V2] ID: {detection.id} | Class: {detection.class_name} | '
                f'Score: {detection.score:.2f} | '
                f'Pixel: ({u:.1f}, {v:.1f}) | '
                f'Scale: {scale_factor:.4f} mm/px | '
                f'World: ({X_mm:.1f}mm, {Y_mm:.1f}mm, {Z_mm:.1f}mm)'
            )

            self.publish_weed_tf(detection.id, X_mm, Y_mm, Z_mm, msg.header.stamp)
            self.get_robot_to_weed_vector(detection.id)

    def publish_weed_tf(self, weed_id, X_mm, Y_mm, Z_mm, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'camera'
        tf_msg.child_frame_id = f'weed_v2_{weed_id}'

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