from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
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

        # CARLA GT odometry: /carla/hero/odometry(map frame) → odom→hero TF
        # 초기 위치를 기준으로 상대 위치를 계산해 odom 프레임으로 발행
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
                '--x', '0.1', '--y', '0', '--z', '0.6',
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
                'min_height': -0.50,
                'max_height': 0.10,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.2,
                'range_max': 12.0,
            }],
            remappings=[
                ('cloud_in', '/carla/hero/lidar'),
                ('scan', '/scan'),
            ],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': True}],
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
                'qos_overrides./scan.subscriber.reliability': 'best_effort',
                'publish_map_to_odom_transform': False,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 60.0,
                'min_laser_range': 0.2,
                'max_laser_range': 12.0,
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.35,
                'do_loop_closing': True,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45,
                'loop_search_maximum_distance': 5.0,
                
                # 스캔 매칭 정밀도
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.005,   # 작을수록 정밀 (기본 0.01)
                'correlation_search_space_smear_deviation': 0.05,

                # 저품질 매칭 거부
                'link_match_minimum_response_fine': 0.2,        # 기본 0.1 → 높이면 불확실한 매칭 버림
                'link_scan_maximum_distance': 1.0,
            }],
            remappings=[
                ('scan', '/scan'),
            ],
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
