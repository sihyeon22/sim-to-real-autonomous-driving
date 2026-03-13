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
                EnvironmentVariable('HOME'), 'CARLA_ws', 'maps', 'realtime_slam_map_ver3.yaml'
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
            description='Convert /cmd_vel to /carla/hero/vehicle_control_cmd',
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
                'odom_frame': 'odom',
                'base_frame': 'hero',
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
                'transform_tolerance': 0.5,
                'min_height': -1.8,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'max_height': 0.0,
                'angle_increment': 0.0087,
                'range_min': 0.3,
                'range_max': 50.0,
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
            package='my_pkg',
            executable='cmd_vel_to_vehicle_control',
            name='cmd_vel_to_vehicle_control',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/carla/hero/vehicle_control_cmd',
                'manual_override_topic': '/carla/hero/vehicle_control_manual_override',
                'max_speed': 3.0,
                'max_throttle': 0.35,
                'max_steer': 0.35,
                'steer_gain': 0.6,
                'brake_on_stop': 0.3,
                'stop_speed_threshold': 0.05,
                'min_drive_throttle': 0.18,
            }],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
