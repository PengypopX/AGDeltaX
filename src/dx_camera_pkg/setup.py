from setuptools import setup
import os
from glob import glob

package_name = 'dx_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # simpler than find_packages if you only have one top-level folder
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
    description='Camera package for ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # e.g. 'camera_node = dx_camera_pkg.camera_node:main'
        ],
    },
)
