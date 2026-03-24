#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

class YoloTranslator(Node):
    def __init__(self):
        super().__init__('yolo_translator')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d_fast',
            self.detection_callback,
            10
        )
        self.get_logger().info('yolo_translator is publishing 3D coordinates.')

    def get_robot_to_object_vector(self, object_id):
        try:
            object_tf = self.tf_buffer.lookup_transform(
                'robot_home',
                f'object_{object_id}',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            dx = object_tf.transform.translation.x * 1000.0
            dy = object_tf.transform.translation.y * 1000.0
            gcode = f'G1 X{int(dx)} Y{int(dy)} Z-800'
            self.get_logger().info(
                f'Object {object_id} -> robot_home: X={dx:.1f}mm Y={dy:.1f}mm | {gcode}'
            )
            return dx, dy, gcode
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None, None, None

    def detection_callback(self, msg):
        for i, detection in enumerate(msg.detections):
            x = detection.bbox3d.center.position.x
            y = detection.bbox3d.center.position.y
            z = detection.bbox3d.center.position.z


            if x == 0.0 and y == 0.0 and z == 0.0:
                continue


            object_id = detection.id if detection.id != '' else f'{detection.class_name}_{i}'


            self.get_logger().info(
                f'ID: {object_id} | Class: {detection.class_name} | '
                f'Score: {detection.score:.2f} | '
                f'World: ({x:.3f}m, {y:.3f}m, {z:.3f}m)'
            )


            self.publish_object_tf(object_id, x, y, z, msg.header.stamp)
            self.get_robot_to_object_vector(object_id)


    def publish_object_tf(self, object_id, x, y, z, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'camera_color_optical_frame'
        tf_msg.child_frame_id = f'object_{object_id}'

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


