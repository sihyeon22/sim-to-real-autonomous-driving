# File: fr09_icp_slam.launch.py
# Description:
#   Kiss-ICP + EKF 기반 FR09 SLAM 맵 생성 launch 파일
#   Kiss-ICP: LiDAR de-skewing + LiDAR odometry 생성
#   EKF: LiDAR odometry + wheel odometry 융합 → 최종 odom 발행
#   slam_toolbox: de-skewed 2D 스캔 + EKF odom으로 맵 생성
#
#   TF 구조:
#   map -> odom (slam_toolbox) -> base_link (EKF) -> hero/lidar, hero/imu (static)
#
#   토픽 흐름:
#   /carla/hero/lidar → Kiss-ICP → /kiss_icp/odometry → EKF → /odometry/filtered
#                                → /kiss_icp/frame    → pointcloud_to_laserscan → /scan → slam_toolbox
#   /odom_wheel (speed_imu_odom) → EKF
#   /carla/hero/imu              → EKF

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


    # TF: base_link -> 센서 프레임 (정적 변환)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar_tf',
        arguments=[
            '--x', '0.456', '--y', '0', '--z', '1.135',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'hero/lidar',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.5',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'hero/imu',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )


    # 휠 오도메트리: speedometer + IMU dead reckoning → /odom_wheel
    # publish_tf=False: odom->base_link TF는 EKF가 담당
    speed_imu_odom = Node(
        package='my_pkg',
        executable='speed_imu_odom',
        name='speed_imu_odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': False,
            'use_imu_angular_velocity': True,
            'invert_speed': False,
        }],
        remappings=[
            ('/hero/speedometer', '/carla/hero/speedometer'),
            ('/hero/imu',         '/carla/hero/imu'),
            ('/hero/wheel_odom',  '/odom_wheel'),
        ],
    )


    # Kiss-ICP: 3D LiDAR → de-skewed PointCloud + LiDAR odometry
    # publish_odom_tf=False: odom->base_link TF는 EKF가 담당
    # lidar_odom_frame=odom: EKF가 참조하는 odom 프레임과 일치
    # publish_debug_clouds=True: /kiss_icp/frame (de-skewed) 토픽 발행 활성화
    kiss_icp = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('my_pkg'), 'config', 'icp', 'kiss_icp_config.yaml']),
            {
                'use_sim_time': True,
                'base_frame': 'base_link',
                'lidar_odom_frame': 'odom',
                'publish_odom_tf': False,
                'publish_debug_clouds': True,
            },
        ],
        remappings=[
            ('pointcloud_topic', '/carla/hero/lidar'),
        ],
    )


    # EKF: /kiss_icp/odometry + /odom_wheel + /carla/hero/imu 융합
    # 출력: /odometry/filtered, odom->base_link TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('my_pkg'), 'config', 'icp', 'ekf_icp.yaml']),
            {'use_sim_time': True},
        ],
    )


    # Kiss-ICP de-skewed 포인트클라우드 → 2D LaserScan 변환
    # cloud_in: /kiss_icp/frame (de-skewed) → 코너 주행 시 스캔 왜곡 해소
    pointcloud_to_laserscan = Node(
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
            'angle_max':  3.14159,
            'angle_increment': 0.0087,
            'range_min': 0.19,
            'range_max': 12.0,
            'qos_overrides./scan.subscriber.reliability': 'best_effort',
        }],
        remappings=[
            ('cloud_in', '/kiss/frame'),
            ('scan',     '/scan'),
        ],
    )


    # slam_toolbox: EKF odom TF + 2D scan → 맵 생성
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'publish_map_to_odom_transform': True,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 60.0,
            'min_laser_range': 0.19,
            'max_laser_range': 12.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.35,
            'do_loop_closing': True,
            # pointcloud_to_laserscan의 BEST_EFFORT QoS와 맞춤
            'qos_overrides./scan.subscriber.reliability': 'best_effort',
        }],
        remappings=[
            ('scan', '/scan'),
        ],
    )


    # 시각화
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )


    return LaunchDescription(ARGUMENTS + [
        lidar_tf,                   # TF
        imu_tf,
        speed_imu_odom,             # 휠 odometry
        kiss_icp,                   # LiDAR odometry + de-skewing
        ekf_node,                   # EKF 융합
        pointcloud_to_laserscan,    # 2D 스캔 변환
        slam_toolbox,               # 맵 생성
        rviz2,                      # 시각화
    ])
