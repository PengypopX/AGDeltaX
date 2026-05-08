#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

class HomographyGenerator(Node):
    def __init__(self):
        super().__init__('homography_generator')

        self.declare_parameter('intrinsics_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/intrinsics.yaml')
        self.declare_parameter('extrinsics_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml')
        self.declare_parameter('output_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/homography.yaml')

        intrinsics_path = self.get_parameter('intrinsics_path').value
        extrinsics_path = self.get_parameter('extrinsics_path').value
        output_path = self.get_parameter('output_path').value

        self.generate(intrinsics_path, extrinsics_path, output_path)

    def generate(self, intrinsics_path, extrinsics_path, output_path):
        # Load intrinsics
        with open(intrinsics_path, 'r') as f:
            intr = yaml.safe_load(f)
        K = np.array(intr['camera_matrix']['data']).reshape(3, 3)

        # Load extrinsics
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

        # Compute homography
        H = K @ np.column_stack([R[:, 0], R[:, 1], t])
        H_inv = np.linalg.inv(H)

        output = {
            'cam2world': {
                'rows': 3,
                'cols': 3,
                'dt': 'd',
                'data': H_inv.flatten().tolist()
            },
            'world2cam': {
                'rows': 3,
                'cols': 3,
                'dt': 'd',
                'data': H.flatten().tolist()
            }
        }

        with open(output_path, 'w') as f:
            yaml.dump(output, f)

        self.get_logger().info(f'homography.yaml saved to {output_path}')
        self.get_logger().info(f'cam2world:\n{H_inv}')
        self.get_logger().info(f'world2cam:\n{H}')

def main(args=None):
    rclpy.init(args=args)
    node = HomographyGenerator()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()