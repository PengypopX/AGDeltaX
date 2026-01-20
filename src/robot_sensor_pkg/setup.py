from setuptools import find_packages, setup

package_name = 'robot_sensor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='fresnostate',
    maintainer_email='fresnostate@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "robot_sensor = robot_sensor_pkg.connection:main",
            "dxon = robot_sensor_pkg.dxon:main",
            "dxon_client_test = robot_sensor_pkg.dxon_client_test:main"
        ],
    },
)
