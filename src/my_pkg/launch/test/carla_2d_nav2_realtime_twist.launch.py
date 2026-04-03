# File: carla_2d_nav2_realtime_twist.launch.py
# Author: sihyeon
# Date: 2026.03.20 FRI
# Description:
#     - 기존 baseline: /cmd_vel -> /ackermann_cmd
#     - carla_twist_to_control 패키지 사용해서 /cmd_vel 변환: /cmd_vel -> /
#     - 두 odometry를 병렬 발행해 dead reckoning 정확도 비교 검증용

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
    use_twist_to_control = LaunchConfiguration('use_twist_to_control')

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
            'use_twist_to_control',
            default_value='true',
            description='Convert /cmd_vel to CARLA vehicle control using carla_twist_to_control',
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
            condition=IfCondition(use_twist_to_control),
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name='carla_twist_to_control',
            output='screen',
            parameters=[{
                'role_name': 'hero',
                # max_cmd_speed: Nav2 max cmd_vel(0.5 m/s)을 throttle 0.25로 매핑
                # → throttle 0.125(max_cmd_speed=4.0)에서 dead zone 확인 → 2.0으로 상향
                # 산출 근거: throttle=0.5 → 실측 1.91 m/s, dead zone 감안 throttle≈0.25 목표
                'max_cmd_speed': 2.2,
                # min_forward_throttle: dead zone 방지를 위해 0.15 유지
                'min_forward_throttle': 0.15,
            }],
            remappings=[
                ('/carla/hero/twist', '/cmd_vel'),
            ],
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
