# File: carla_2d_nav2_wheel_odom.launch.py
# Author: sihyeon
# Date: 2026.03.24 MON
# Description:
#     - speedometer + IMU dead reckoning 기반 /odom_wheel을 Nav2 odometry로 직접 연결
#     - /odom_local 없이 실차 전환 가능한 센서만으로 Nav2 주행
#     - odom_local baseline 대비 localization 및 주행 성능 비교용 실험 baseline

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    nav2_params = LaunchConfiguration('nav2_params')
    use_rviz = LaunchConfiguration('use_rviz')
    use_cmd_vel_relay = LaunchConfiguration('use_cmd_vel_relay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'CARLA_ws', 'maps', 'realtime_slam_map_blocked.yaml'
            ]),
            description='Full path to saved map yaml',
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_pkg'), 'config', 'nav2_params_wheel_odom.yaml'
            ]),
            description='Full path to Nav2 params yaml (wheel odom version)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with Nav2 default config',
        ),
        DeclareLaunchArgument(
            'use_cmd_vel_relay',
            default_value='true',
            description='Convert /cmd_vel to /carla/hero/ackermann_cmd',
        ),

        # Realtime mode: expects CARLA bridge topics (/clock, /carla/hero/*) to be already available.
        # speed+IMU dead reckoning odometry — odom->hero TF 직접 발행 (Nav2에 연결)
        Node(
            package='my_pkg',
            executable='speed_imu_odom',
            name='speed_imu_odom_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'hero',
                'publish_tf': True,
                'use_imu_angular_velocity': True,
                'invert_speed': False,
            }],
            remappings=[
                ('/hero/speedometer', '/carla/hero/speedometer'),
                ('/hero/imu',         '/carla/hero/imu'),
                ('/hero/wheel_odom',  '/odom_wheel'),
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hero_to_lidar_static_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '2.4',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'hero', '--child-frame-id', 'hero/lidar',
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hero_to_imu_static_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'hero', '--child-frame-id', 'hero/imu',
            ],
            output='screen',
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_inf': True,
                'target_frame': 'hero/lidar',
                'transform_tolerance': 1.2,
                'min_height': -1.8,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'max_height': 0.0,
                'angle_increment': 0.05,
                'range_min': 0.3,
                'range_max': 30.0,
            }],
            remappings=[
                ('cloud_in', '/carla/hero/lidar'),
                ('scan', '/scan'),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
            ),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'params_file': nav2_params,
                'use_sim_time': 'True',
                'autostart': 'True',
                'use_composition': 'False',
            }.items(),
        ),

        Node(
            condition=IfCondition(use_cmd_vel_relay),
            package='carla_ackermann_control',
            executable='carla_ackermann_control_node',
            name='carla_ackermann_control',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('carla_ackermann_control'),
                    'config',
                    'settings.yaml',
                ]),
                {
                    'role_name': 'hero',
                    'control_loop_rate': 0.05,
                }
            ],
        ),

        Node(
            condition=IfCondition(use_cmd_vel_relay),
            package='my_pkg',
            executable='cmd_vel_to_ackermann',
            name='cmd_vel_to_ackermann',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/carla/hero/ackermann_cmd',
                'wheelbase': 3.0,
                'max_steering_angle': 0.6,
                'angular_deadband': 0.06,
                'min_speed_for_steer': 0.1,
                'max_speed': 5.0,
                'acceleration': 0.8,
                'jerk': 0.0,
            }],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('my_pkg'), 'rviz', 'carla.rviz'])],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
