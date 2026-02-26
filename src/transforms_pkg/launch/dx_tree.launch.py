import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Load extrinsics (camera -> charuco_board)
    with open('/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics.yaml', 'r') as f:
        extr = yaml.safe_load(f)

    # Load extrinsics_inv (robot_arm_base -> camera_frame)
    with open('/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics_inv.yaml', 'r') as f:
        extr_inv = yaml.safe_load(f)

    t = extr['translation']
    r = extr['rotation']

    t_inv = extr_inv['translation']
    r_inv = extr_inv['rotation']

    return LaunchDescription([
        # world -> robot_arm_base (50cm ground offset)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '-0.5', '0', '0', '0', '1', 'robot_home', 'floor']
        ),
        # robot_arm_base -> camera_frame (inverted extrinsics)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                str(t_inv['x']), str(t_inv['y']), str(t_inv['z']),
                str(r_inv['x']), str(r_inv['y']), str(r_inv['z']), str(r_inv['w']),
                'floor', 'camera'
            ]
        ),
        # camera_frame -> charuco_board (original extrinsics)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                str(t['x']), str(t['y']), str(t['z']),
                str(r['x']), str(r['y']), str(r['z']), str(r['w']),
                'camera', 'floor'
            ]
        ),
        # yolo pixel to world transform node
        Node(
            package='transforms_pkg',
            executable='yolo_pixel_to_mm',
            name='yolo_pixel_to_mm'
        ),
    ])