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

        # Custom dead reckoning odometry: speedometer + IMU → odom→hero TF
        # use_imu_angular_velocity=False: CARLA IMU orientation을 직접 사용 (GT yaw)
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
                'use_msg_stamp': True,
            }],
        ),
        
        # Keep only the LiDAR mounting TF needed for 2D scan conversion.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hero_to_lidar_static_tf',
            arguments=[
                '--x', '0.456', '--y', '0', '--z', '1.135',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'hero', '--child-frame-id', 'hero/lidar',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # # 프레임 누적: 최근 N 프레임을 합쳐서 sparsity 완화
        # Node(
        #     package='my_pkg',
        #     executable='pointcloud_accumulator',
        #     name='pointcloud_accumulator',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': True,
        #         'buffer_size': 2,
        #         'input_topic':  '/carla/hero/lidar',
        #         'output_topic': '/lidar/accumulated',
        #     }],
        # ),

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
                'min_height': -1.05,
                'max_height': 0.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.19,
                'range_max': 10.0,
                'qos_overrides./scan.subscriber.reliability': 'best_effort',
            }],
            remappings=[
                ('cloud_in', '/carla/hero/lidar'),
                ('scan', '/scan'),
            ],
        ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '0', '--pitch', '0', '--yaw', '0',
        #         '--frame-id', 'map', '--child-frame-id', 'odom',
        #     ],
        #     parameters=[{'use_sim_time': True}],
        # ),
        
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
                'publish_map_to_odom_transform': True,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 60.0,
                'min_laser_range': 0.19,
                'max_laser_range': 10.0,
                # 'minimum_travel_distance': 0.2,
                # 'minimum_travel_heading': 0.2,
                'do_loop_closing': True,
                
                # 'throttle_scans': 2,
                # 'minimum_time_interval': 0.1,
                # 'map_update_interval': 1.0,

                # 'loop_match_minimum_response_coarse': 0.35,
                # 'loop_match_minimum_response_fine': 0.45,
                # 'loop_search_maximum_distance': 5.0,
                
                # # 스캔 매칭 정밀도
                # 'correlation_search_space_dimension': 0.5,
                # 'correlation_search_space_resolution': 0.01,   # 작을수록 정밀 (기본 0.01)
                # 'correlation_search_space_smear_deviation': 0.05,

                # # 저품질 매칭 거부
                # 'link_match_minimum_response_fine': 0.2,        # 기본 0.1 → 높이면 불확실한 매칭 버림
                # 'link_scan_maximum_distance': 1.0,
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
