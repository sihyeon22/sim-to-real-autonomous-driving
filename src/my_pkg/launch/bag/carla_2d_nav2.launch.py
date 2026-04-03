from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    map_yaml = LaunchConfiguration('map_yaml')
    nav2_params = LaunchConfiguration('nav2_params')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'my_ws', 'bag']),
            description='Path to rosbag2 directory',
        ),
        DeclareLaunchArgument(
            'bag_rate',
            default_value='1.0',
            description='Rosbag playback rate',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'CARLA_ws', 'maps', 'lidar_launch.yaml'
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

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', bag_path, '-r', bag_rate, '-l',
                '--topics',
                '/clock',
                '/carla/hero/lidar',
                '/carla/hero/odometry',
                '/tf_static',
            ],
            output='screen',
        ),

        Node(
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
                'target_frame': 'hero/lidar',
                'transform_tolerance': 0.2,
                'min_height': -0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'max_height': 0.5,
                'angle_increment': 0.0087,
                'range_min': 1.0,
                'range_max': 50.0,
                'use_inf': False,
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
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
