from setuptools import find_packages, setup
from glob import glob

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dgist',
    maintainer_email='dgist@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_ackermann = my_pkg.cmd_vel_to_ackermann:main',
            'speed_imu_odom = my_pkg.odometry:main',
            'waypoint_sender = my_pkg.waypoint_sender:main',
            'sensor_bridge = my_pkg.sensor_bridge:main',
        ],
    },
)
