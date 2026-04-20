# File: mkmini_modular.launch.py
# Author: sihyeon
# Description:
#     MK-Mini 자율주행 모듈형 통합 런치
#     각 모듈은 단독 실행 가능하며, 공통 nav2_params_mkmini.yaml 참조
#
#     Pipeline:
#       perception → localization → planner → controller → navigator
#
#     단독 실행 예시:
#       ros2 launch my_pkg perception.launch.py
#       ros2 launch my_pkg localization.launch.py map_yaml:=<path>
#       ros2 launch my_pkg planner.launch.py
#       ros2 launch my_pkg controller.launch.py
#       ros2 launch my_pkg navigator.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml              = LaunchConfiguration('map_yaml')
    nav2_params           = LaunchConfiguration('nav2_params')
    use_sim_time          = LaunchConfiguration('use_sim_time')
    use_rviz              = LaunchConfiguration('use_rviz')
    use_vehicle_interface = LaunchConfiguration('use_vehicle_interface')

    launch_dir = PathJoinSubstitution([FindPackageShare('my_pkg'), 'launch'])

    def include_module(filename, extra_args=None):
        args = {
            'use_sim_time': use_sim_time,
            'nav2_params':  nav2_params,
        }
        if extra_args:
            args.update(extra_args)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([launch_dir, filename])
            ),
            launch_arguments=args.items(),
        )

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
                FindPackageShare('my_pkg'), 'config', 'nav2_params_mkmini.yaml'
            ]),
            description='Nav2 params yaml',
        ),
        DeclareLaunchArgument('use_sim_time',          default_value='true'),
        DeclareLaunchArgument('use_rviz',              default_value='true'),
        DeclareLaunchArgument('use_vehicle_interface', default_value='true',
                              description='Launch cmd_vel_to_ackermann + carla_ackermann'),

        # ── Module 1: Perception ─────────────────────────────────────────────
        include_module('perception.launch.py'),

        # ── Module 2: Localization ───────────────────────────────────────────
        include_module('localization.launch.py', {'map_yaml': map_yaml}),

        # ── Module 3: Planner ────────────────────────────────────────────────
        include_module('planner.launch.py'),

        # ── Module 4: Controller ─────────────────────────────────────────────
        include_module('controller.launch.py'),

        # ── Module 5: Navigator ──────────────────────────────────────────────
        include_module('navigator.launch.py'),

        # ── Vehicle Interface (CARLA sim) ────────────────────────────────────
        Node(
            condition=IfCondition(use_vehicle_interface),
            package='carla_ackermann_control',
            executable='carla_ackermann_control_node',
            name='carla_ackermann_control',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('carla_ackermann_control'), 'config', 'settings.yaml',
                ]),
                {'role_name': 'hero', 'control_loop_rate': 0.05},
            ],
        ),

        Node(
            condition=IfCondition(use_vehicle_interface),
            package='my_pkg',
            executable='cmd_vel_to_ackermann',
            name='cmd_vel_to_ackermann',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/carla/hero/ackermann_cmd',
                'wheelbase': 0.6,
                'max_steering_angle': 0.5,
                'angular_deadband': 0.06,
                'min_speed_for_steer': 0.1,
                'max_speed': 5.0,
                'acceleration': 0.0,
                'jerk': 0.0,
            }],
        ),

        # ── RViz ─────────────────────────────────────────────────────────────
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_pkg'), 'rviz', 'carla.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
