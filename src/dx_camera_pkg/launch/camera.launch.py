from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve the path to your calibration YAML inside the package
    pkg_share = get_package_share_directory('dx_camera_pkg')
    calibration_file = os.path.join(pkg_share, 'config', 'camera_calibration.yaml')

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',   # ROS 2 executable name for usb_cam
            name='usb_cam',
            output='screen',
            parameters=[{
                'camera_name': 'usb_cam',
                'camera_info_url': f'file://{calibration_file}',
                'video_device': '/dev/video0',
                'image_width': 1280,
                'image_height': 720,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_frame_id': 'camera'
            }]

        )
    ])
