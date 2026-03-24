from setuptools import find_packages, setup
import os 
from glob import glob
package_name = 'transforms_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fresnostate',
    maintainer_email='brandonjwilbur@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_subscriber = transforms_pkg.ros_transformer:main',
            'translate_to_3d = transforms_pkg.translate_to_3d:main',
            'invert_yaml = transforms_pkg.invert_yaml:main',
            'homography_generator = transforms_pkg.homography_generator:main',
            'yolo_translator = transforms_pkg.yolo_translator:main'
        ],
    },
)
