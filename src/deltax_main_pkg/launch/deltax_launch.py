from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # robot_base -> floor
        Node(
            package='deltax_main_pkg',
            executable='robot_poll',
            name='robot_poll'
        ),
        Node(
            package='deltax_main_pkg',
            executable='send_ts',
            name='send_ts'
        ),
        Node(
            package='deltax_main_pkg',
            executable='claw',
            name='claw'
        ),
    ])




