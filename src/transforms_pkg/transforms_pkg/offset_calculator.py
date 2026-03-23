#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
import yaml
import numpy as np


class CoinCalibrator(Node):
    def __init__(self):
        super().__init__('coin_calibrator')
        self.samples = []
        self.target_samples = 30  # average over 30 frames for stability


        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.detection_callback,
            10
        )
        self.get_logger().info('Place coin at known position. Collecting samples...')


    def detection_callback(self, msg):
        for detection in msg.detections:
            x = detection.bbox3d.center.position.x
            y = detection.bbox3d.center.position.y
            z = detection.bbox3d.center.position.z


            if x == 0.0 and y == 0.0 and z == 0.0:
                continue


            self.samples.append([x, y, z])
            self.get_logger().info(f'Sample {len(self.samples)}/{self.target_samples}')


            if len(self.samples) >= self.target_samples:
                self.save_calibration()
                rclpy.shutdown()


    def save_calibration(self):
        avg = np.mean(self.samples, axis=0)
        self.get_logger().info(f'Averaged position: X={avg[0]:.4f} Y={avg[1]:.4f} Z={avg[2]:.4f}')


        # where is coin relative to deltax base
        known_x = 0.0  # meters from robot_home
        known_y = 0.0
        known_z = -.5  # 50cm ground offset


        offset = {
            'measured': {'x': float(avg[0]), 'y': float(avg[1]), 'z': float(avg[2])},
            'known':    {'x': known_x, 'y': known_y, 'z': known_z},
            'offset':   {
                'x': float(known_x - avg[0]),
                'y': float(known_y - avg[1]),
                'z': float(known_z - avg[2])
            }
        }


        path = '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/coin_offset.yaml'
        with open(path, 'w') as f:
            yaml.dump(offset, f)
        self.get_logger().info(f'Saved to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = CoinCalibrator()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()




