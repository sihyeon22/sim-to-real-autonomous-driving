from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'my_ws', 'bag']),
            description='Path to rosbag2 directory',
        ),
        DeclareLaunchArgument(
            'bag_rate',
            default_value='0.3',
            description='Rosbag playback rate',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([FindPackageShare('my_pkg'), 'rviz', 'carla_slam.rviz']),
            description='RViz2 config file',
        ),

        # Replay bag-recorded /clock only (do not use --clock) to avoid duplicate publishers.
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

        # Convert absolute map->hero pose (odometry) into relative odom->hero TF.
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

        # Keep only the LiDAR mounting TF needed for 2D scan conversion.
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
                'target_frame': 'hero',
                'transform_tolerance': 0.2,
                'min_height': -0.1,
                'angle_min': -3.14159,
                'max_height': 0.2,
                'angle_max': 3.14159,
                'angle_increment': 0.0174,
                'range_min': 1.0,
                'range_max': 15.0,
                'use_inf': False,
            }],
            remappings=[
                ('cloud_in', '/carla/hero/lidar'),
                ('scan', '/scan'),
            ],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'hero',
                'scan_topic': '/scan',
                'publish_map_to_odom_transform': True,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
            }],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
