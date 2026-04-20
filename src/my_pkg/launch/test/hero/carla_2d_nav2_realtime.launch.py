# File: carla_2d_nav2_realtime.launch.py
# Author: sihyeon
# Date: 2026.03.16 MON
# Description:
#     - CARLA Nav2 Ackermann 주행 기본 baseline
#     - /odom_local: CARLA ground truth odometry 기반 (실차 전환 불가)
#     - Nav2 localization 성능의 이론적 상한선(upper bound)으로 활용
#     - 비교 실험의 레퍼런스 기준으로 사용

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
    use_relative_odom_tf = LaunchConfiguration('use_relative_odom_tf')
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
                FindPackageShare('my_pkg'), 'config', 'nav2_params.yaml'
            ]),
            description='Full path to Nav2 params yaml',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with Nav2 default config',
        ),
        DeclareLaunchArgument(
            'use_relative_odom_tf',
            default_value='true',
            description='Enable odom_tf_from_map_pose (odom->hero from /carla/hero/odometry)',
        ),
        DeclareLaunchArgument(
            'use_cmd_vel_relay',
            default_value='true',
            description='Convert /cmd_vel to /carla/hero/ackermann_cmd',
        ),

        # Realtime mode: expects CARLA bridge topics (/clock, /carla/hero/*) to be already available.
        Node(
            condition=IfCondition(use_relative_odom_tf),
            package='my_pkg',
            executable='odom_tf_from_map_pose',
            name='odom_tf_from_map_pose',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/carla/hero/odometry',
                'output_odom_topic': '/odom_local',
                'odom_frame': 'odom',
                'base_frame': 'hero',
                'publish_rate': 30.0,
                'use_msg_stamp': False,
            }],
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
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_inf': True,
                'target_frame': 'hero/lidar',
                # 'transform_tolerance': 1.2,
                # 'min_height': -1.8,
                # 'angle_min': -3.14159,
                # 'angle_max': 3.14159,
                # 'max_height': 0.0,
                # 'angle_increment': 0.05,
                # 'range_min': 0.3,
                # 'range_max': 30.0,
                'transform_tolerance': 2.0,
                'min_height': -2.35,
                'max_height': -1.8,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.3,
                'range_max': 15.0,
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
