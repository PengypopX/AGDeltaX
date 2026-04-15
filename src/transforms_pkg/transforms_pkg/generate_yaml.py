#!/usr/bin/env python3
"""
generate_extrinsics.py
----------------------
Reads IMU accelerometer for roll/pitch, takes X/Y/Z/yaw as
command line arguments, saves extrinsics.yaml.


Usage:
  ros2 run transforms_pkg generate_yaml --ros-args -p x:=-0.108 -p y:=-0.001 -p z:=0.457 -p yaw:=-1.4
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


EXTRINSICS_PATH = '/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml'
NUM_SAMPLES     = 100


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)




class GenerateExtrinsics(Node):


    def __init__(self):
        super().__init__('generate_extrinsics')


        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('x',   -0.108)
        self.declare_parameter('y',   -0.001)
        self.declare_parameter('z',    0.457)
        self.declare_parameter('yaw', -1.4)


        self._tx  = self.get_parameter('x').value
        self._ty  = self.get_parameter('y').value
        self._tz  = self.get_parameter('z').value
        self._yaw = self.get_parameter('yaw').value


        self._accel_samples = []


        self._accel_sub = self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self._accel_cb,
            _SENSOR_QOS,
        )


        self.get_logger().info(
            f'Translation: x={self._tx}  y={self._ty}  z={self._tz}'
        )
        self.get_logger().info(
            f'Yaw: {self._yaw}° (from parameter)'
        )
        self.get_logger().info(
            f'Collecting {NUM_SAMPLES} accelerometer samples — keep camera still...'
        )


    def _accel_cb(self, msg: Imu):
        if len(self._accel_samples) >= NUM_SAMPLES:
            return


        self._accel_samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])


        self.get_logger().info(
            f'Sample {len(self._accel_samples)}/{NUM_SAMPLES}',
            throttle_duration_sec=1.0
        )


        if len(self._accel_samples) == NUM_SAMPLES:
            self._generate()


    def _generate(self):
        # ── Roll / pitch from averaged accelerometer ──────────────────────────
        arr = np.array(self._accel_samples)
        ax, ay, az = arr.mean(axis=0)


        roll  = np.degrees(np.arctan2(ay, az))
        pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
        yaw   = self._yaw


        self.get_logger().info(
            f'IMU  roll={roll:.3f}°  pitch={pitch:.3f}°  yaw={yaw:.3f}°'
        )


        # ── Build rotation quaternion ──────────────────────────────────────────
        r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        q = r.as_quat()  # xyzw


        # ── Save extrinsics.yaml (camera_link -> floor) ───────────────────────
        extrinsics = {
            'camera_frame': 'camera_link',
            'charuco_frame': 'floor',
            'euler_degrees': {
                'roll':  float(roll),
                'pitch': float(pitch),
                'yaw':   float(yaw),
            },
            'rotation': {
                'x': float(q[0]),
                'y': float(q[1]),
                'z': float(q[2]),
                'w': float(q[3]),
            },
            'translation': {
                'x': self._tx,
                'y': self._ty,
                'z': self._tz,
            }
        }


        with open(EXTRINSICS_PATH, 'w') as f:
            yaml.dump(extrinsics, f, default_flow_style=False)


        self.get_logger().info(f'Saved {EXTRINSICS_PATH}')
        self.get_logger().info('Run your inversion script next, then restart transforms launch.')


        rclpy.shutdown()




def main(args=None):
    rclpy.init(args=args)
    node = GenerateExtrinsics()
    rclpy.spin(node)




if __name__ == '__main__':
    main()




