# File: mkmini_2d_slam_real.launch.py
# Author: sihyeon
# Date: 2026.04.17 FRI
# Description:
#     MK-Mini3 실차 기준 2D SLAM 런치 파일
#     - 실차 표준 토픽/프레임 사용 (Ouster OS1 기준)
#     - GT odometry 제거, speed_imu_odom (speedometer + IMU) 기반 custom odometry 사용
#
#     [구조]
#     ┌────────────────────────────────────────────────────────────┐
#     │  CARLA Bridge Layer (Need to remove in real env)           │
#     │  - sensor_bridge: 센서 토픽 → /raw/points, /raw/imu  │
#     │  - speed_imu_odom: speedometer + IMU → /odom               │
#     └────────────────────────────────────────────────────────────┘
#     ┌────────────────────────────────────────────────────────────┐
#     │  Unified Pipeline                                          │
#     │  - TF: base_link → os_sensor, os_imu                       │
#     │  - speed_imu_odom: /odom + odom→base_link TF publish       │
#     │  - pointcloud_to_laserscan: /raw/points → /scan            │
#     │  - slam_toolbox: /scan + /odom → /map                      │
#     └────────────────────────────────────────────────────────────┘
#
#     [실차 배포 시 교체 목록]
#     제거: speed_imu_odom의 CARLA remapping, use_sim_time
#     변경: sensor_bridge
#     추가: ouster_launch.py (→ /raw/points, /raw/imu 발행)
#          yhs_can_control  (→ /odom 발행)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
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
]


def generate_launch_description():
    use_rviz    = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # =========================================================
    # [실차 공통 파이프라인] - 실차/시뮬 동일하게 사용
    # =========================================================

    # TF: base_link → Ouster OS1 표준 프레임
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_sensor_tf',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '0.6',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'os_sensor',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_imu_tf',
        arguments=[
            '--x', '0.1', '--y', '0', '--z', '0.6',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'os_imu',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # custom odometry: speedometer + /raw/imu → /odom + odom→base_link TF
    speed_imu_odom = Node(
        package='my_pkg',
        executable='speed_imu_odom',
        name='speed_imu_odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_tf': True,
            'use_imu_angular_velocity': True,
            'invert_speed': False,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }],
        remappings=[
            ('/hero/speedometer', '/carla/hero/speedometer'),
            ('/hero/imu',         '/raw/imu'),
            ('/hero/wheel_odom',  '/odom'),
        ],
    )

    # /raw/points (PointCloud2) → /scan (LaserScan)
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_inf': True,
            'target_frame': 'os_sensor',
            'transform_tolerance':  2.0,
            'min_height':          -0.50,
            'max_height':          -0.05,
            'angle_min':           -3.14159,
            'angle_max':            3.14159,
            'angle_increment':      0.0087,
            'range_min':            0.3,
            'range_max':            12.0,
            'qos_overrides./scan.subscriber.reliability': 'best_effort',
        }],
        remappings=[
            ('cloud_in', '/raw/points'),
            ('scan',     '/scan'),
        ],
    )

    # SLAM: /scan + odom→base_link TF → /map
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'do_loop_closing': True,
            'publish_map_to_odom_transform': True,
            'map_frame':  'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'qos_overrides./scan.subscriber.reliability': 'best_effort',
            'transform_timeout':  0.2,
            'tf_buffer_duration': 60.0,
            'min_laser_range':    0.3,
            'max_laser_range':    12.0,

            'minimum_travel_distance': 0.30,
            'minimum_travel_heading':  0.55,

            'loop_search_maximum_distance':       5.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine':   0.45,

            'correlation_search_space_resolution':      0.005,
            'correlation_search_space_dimension':       0.6,
            'correlation_search_space_smear_deviation': 0.04,

            'link_match_minimum_response_fine': 0.18,
            'link_scan_maximum_distance':       1.4,
        }],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # =========================================================
    # [실차 전환 시 변경 필요] - lidar_input, imu_input
    # =========================================================
    
    sensor_bridge = Node(
        package='my_pkg',
        executable='sensor_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[{
            'lidar_input':  '/carla/hero/lidar',
            'lidar_output': '/raw/points',
            'lidar_frame':  'os_sensor',
            'imu_input':    '/carla/hero/imu',
            'imu_output':   '/raw/imu',
            'imu_frame':    'os_imu',
        }],
    )

    return LaunchDescription(ARGUMENTS + [
        # 실차 공통
        lidar_tf,
        imu_tf,
        speed_imu_odom,
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz2,
        # 실차 환경에서 수정 필요
        sensor_bridge,
    ])
