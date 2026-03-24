import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Load extrinsics (camera -> charuco_board)
    with open('/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml', 'r') as f:
        extr = yaml.safe_load(f)


    # Load extrinsics_inv (floor -> camera_optical)
    with open('/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics_inv.yaml', 'r') as f:
        extr_inv = yaml.safe_load(f)


    t_inv = extr_inv['translation']
    r_inv = extr_inv['rotation']


    return LaunchDescription([
        # robot_home -> floor (50cm ground offset)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '-0.5', '0', '0', '0', '1', 'robot_home', 'floor']
        ),
        # floor -> camera_optical (inverted extrinsics)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                str(t_inv['x']), str(t_inv['y']), str(t_inv['z']),
                str(r_inv['x']), str(r_inv['y']), str(r_inv['z']), str(r_inv['w']),
                'floor', 'camera_optical'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_optical', 'camera_color_optical_frame']
        )
    ])

