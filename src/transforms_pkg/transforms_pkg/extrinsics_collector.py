import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


class ExtrinsicsCollector(Node):
    def __init__(self):
        super().__init__('extrinsics_collector')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.samples = []
        self.num_samples = 10
        self.output_path = 'extrinsics.yaml'

        self.timer = self.create_timer(1.0, self.collect_sample)
        self.get_logger().info('Collecting 10 TF samples...')

    def collect_sample(self):
        if len(self.samples) >= self.num_samples:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'camera_frame',
                'charuco_board',
                rclpy.time.Time()
            )

            t = tf.transform.translation
            r = tf.transform.rotation

            self.samples.append({
                'xyz': [t.x, t.y, t.z],
                'quat': [r.x, r.y, r.z, r.w]
            })

            self.get_logger().info(f'Sample {len(self.samples)}/{self.num_samples} collected')

            if len(self.samples) == self.num_samples:
                self.save_extrinsics()

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

    def save_extrinsics(self):
        xyz_array = np.array([s['xyz'] for s in self.samples])
        quat_array = np.array([s['quat'] for s in self.samples])

        # Average XYZ
        avg_xyz = np.mean(xyz_array, axis=0).tolist()

        # Convert quaternions to euler angles, take median, convert back
        eulers = Rotation.from_quat(quat_array).as_euler('xyz', degrees=True)
        median_euler = np.median(eulers, axis=0)
        median_quat = Rotation.from_euler('xyz', median_euler, degrees=True).as_quat().tolist()

        extrinsics = {
            'camera_frame': 'camera_frame',
            'charuco_frame': 'charuco_board',
            'translation': {
                'x': avg_xyz[0],
                'y': avg_xyz[1],
                'z': avg_xyz[2]
            },
            'rotation': {
                'x': median_quat[0],
                'y': median_quat[1],
                'z': median_quat[2],
                'w': median_quat[3]
            },
            'euler_degrees': {
                'roll': float(median_euler[0]),
                'pitch': float(median_euler[1]),
                'yaw': float(median_euler[2])
            }
        }

        with open(self.output_path, 'w') as f:
            yaml.dump(extrinsics, f, default_flow_style=False)

        self.get_logger().info(f'Extrinsics saved to {self.output_path}')
        self.get_logger().info(f'  Translation: {avg_xyz}')
        self.get_logger().info(f'  Euler (deg): {median_euler.tolist()}')

        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicsCollector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()