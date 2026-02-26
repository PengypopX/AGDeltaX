#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class ExtrinsicsInverter(Node):
    def __init__(self):
        super().__init__('extrinsics_inverter')
        
        self.declare_parameter('extrinsics_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml')
        self.declare_parameter('output_path', '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics_inv.yaml')
        
        extrinsics_path = self.get_parameter('extrinsics_path').value
        output_path = self.get_parameter('output_path').value
        
        self.invert_and_save(extrinsics_path, output_path)

    def invert_and_save(self, input_path, output_path):
        # Load extrinsics
        with open(input_path, 'r') as f:
            extr = yaml.safe_load(f)

        t = np.array([extr['translation']['x'], extr['translation']['y'], extr['translation']['z']])
        q = [extr['rotation']['x'], extr['rotation']['y'], extr['rotation']['z'], extr['rotation']['w']]

        R = Rotation.from_quat(q)

        # Invert
        R_inv = R.inv()
        t_inv = -(R_inv.as_matrix() @ t)
        q_inv = R_inv.as_quat()
        euler_inv = R_inv.as_euler('xyz', degrees=True)

        # Build output yaml
        output = {
            'camera_frame': extr['charuco_frame'],
            'charuco_frame': extr['camera_frame'],
            'translation': {
                'x': float(t_inv[0]),
                'y': float(t_inv[1]),
                'z': float(t_inv[2])
            },
            'rotation': {
                'x': float(q_inv[0]),
                'y': float(q_inv[1]),
                'z': float(q_inv[2]),
                'w': float(q_inv[3])
            },
            'euler_degrees': {
                'roll': float(euler_inv[0]),
                'pitch': float(euler_inv[1]),
                'yaw': float(euler_inv[2])
            }
        }

        with open(output_path, 'w') as f:
            yaml.dump(output, f, default_flow_style=False)

        self.get_logger().info(f'Inverted extrinsics saved to {output_path}')
        self.get_logger().info(f'Translation: x={t_inv[0]:.6f}, y={t_inv[1]:.6f}, z={t_inv[2]:.6f}')
        self.get_logger().info(f'Quaternion: x={q_inv[0]:.6f}, y={q_inv[1]:.6f}, z={q_inv[2]:.6f}, w={q_inv[3]:.6f}')

def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicsInverter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()