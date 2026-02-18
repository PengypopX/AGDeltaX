from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_sensor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['robot_sensor_pkg', 'deltax']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='fresnostate',
    maintainer_email='fresnostate@todo.todo',
    description='Robot sensor package for ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            "robot_sensor = robot_sensor_pkg.connection:main",
            "dxon = robot_sensor_pkg.dxon:main",
            "dxon_client_test = robot_sensor_pkg.dxon_client_test:main",
            "drive_motor = robot_sensor_pkg.drive_motor:main"
        ],
    },
)
