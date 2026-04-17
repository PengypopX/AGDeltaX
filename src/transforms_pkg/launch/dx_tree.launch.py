import yaml
from launch import LaunchDescription
from launch_ros.actions import Node




def generate_launch_description():
    with open('/home/fresnostate/AGDeltaX/src/transforms_pkg/config/extrinsics_inv.yaml', 'r') as f:
        extr_inv = yaml.safe_load(f)


    t = extr_inv['translation']
    r = extr_inv['rotation']


    return LaunchDescription([
        # robot_base -> floor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '-.2', '-.5', '0', '0', '0', '1', 'robot_home', 'floor'] #hard coded offset of 20cm
        ),
        # floor -> camera_link (inverted extrinsics)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                str(t['x']), str(t['y']), str(t['z']),
                str(r['x']), str(r['y']), str(r['z']), str(r['w']),
                'floor', 'camera_link'
            ]
        ),
    ])




