from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # robot_base -> floor
        Node(
            package='transforms_pkg',
            executable='robot_poll',
            name='robot_poll'
        ),
        Node(
            package='transforms_pkg',
            executable='send_ts',
            name='send_ts'
        ),
        Node(
            package='transforms_pkg',
            executable='claw',
            name='claw'
        ),
    ])




