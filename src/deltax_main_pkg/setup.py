from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'deltax_main_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['deltax_main_pkg', 'deltax']),
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
    description='deltax ros intermediate',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            "send_ts = deltax_main_pkg.send_ts:main",
            "robot_poll = deltax_main_pkg.robot_poll:main",
            "claw = deltax_main_pkg.claw:main"
        ],
    },
)
