# File: baseline_fr09_real.launch.py
# Author: sihyeon
# Date: 2026.04.16 WED
# Description:
#     FR-09 실차 기준 Nav2 베이스라인
#     - 실차 표준 토픽/프레임 이름 사용 (Ouster OS1 기준)
#     - /raw/points, /raw/imu, /odom, os_sensor, os_imu
#
#     [구조]
#     ┌────────────────────────────────────────────────────────────┐
#     │  CARLA Bridge Layer (Need to fix in real env)              │
#     │  - sensor_bridge: Sensor topic → /raw/points, /raw/imu     │
#     │  - speed_imu_odom: speedometer + IMU → /odom               │
#     │  - cmd_vel_to_ackermann + carla_ackermann_control          │
#     └────────────────────────────────────────────────────────────┘
#     ┌────────────────────────────────────────────────────────────┐
#     │  Unified Pipeline                                          │
#     │  - TF: base_link → os_sensor, os_imu                       │
#     │  - pointcloud_to_laserscan: /raw/points → /scan            │
#     │  - nav2_bringup: /odom, /scan subscribe                    │
#     └────────────────────────────────────────────────────────────┘
#
#     [실차 배포 시 교체 목록]
#     제거: speed_imu_odom, cmd_vel_to_ackermann, carla_ackermann_control, use_sim_time
#     변경: sensor_bridge
#     추가: ouster_launch.py (→ /raw/points, /raw/imu 발행)
#          yhs_can_control  (→ /odom 발행, /cmd_vel 수신 후 CAN 제어)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Launch Arguments
ARGUMENTS = [
    DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'CARLA_ws', 'maps', 'parking', 'real.yaml'
        ]),
        description='Full path to saved map yaml',
    ),
    DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_pkg'), 'config', 'nav2_params_fr09_real.yaml'
        ]),
        description='Full path to Nav2 params yaml (real vehicle naming)',
    ),
    DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2',
    ),
]


def generate_launch_description():
    map_yaml    = LaunchConfiguration('map_yaml')
    nav2_params = LaunchConfiguration('nav2_params')
    use_rviz    = LaunchConfiguration('use_rviz')

    # =========================================================
    # [실차 공통 파이프라인] - 실차/시뮬 동일하게 사용
    # =========================================================

    # TF: base_link -> Ouster OS1 표준 프레임
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_sensor_tf',
        arguments=[
            '--x', '0.456', '--y', '0', '--z', '1.135',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'os_sensor',
        ],
        output='screen',
    )

    # os_imu 프레임: Ouster 내장 IMU는 os_sensor와 동일 위치에 장착
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_imu_tf',
        arguments=[
            '--x', '0.456', '--y', '0', '--z', '1.135',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'os_imu',
        ],
        output='screen',
    )

    # 센서: /raw/points (PointCloud2) -> /scan (LaserScan)
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_inf': True,
            'target_frame': 'os_sensor',        # Ouster 표준 프레임
            'transform_tolerance': 2.0,
            'min_height':         -0.98,
            'max_height':          0.00,
            'angle_min':          -3.14159,
            'angle_max':           3.14159,
            'angle_increment':     0.0087,
            'range_min':           0.19,
            'range_max':           12.0,
        }],
        remappings=[
            ('cloud_in', '/raw/points'),        # 실차: Ouster /raw/points
            ('scan',     '/scan'),
        ],
    )

    # Nav2 bringup (AMCL + planner + controller)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'slam':            'False',
            'map':              map_yaml,
            'params_file':      nav2_params,
            'use_sim_time':    'True',
            'autostart':       'True',
            'use_composition': 'False',
        }.items(),
    )

    # 시각화: RViz2
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('my_pkg'), 'rviz', 'carla.rviz'])],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    # =========================================================
    # [실차 전환 시 수정 필요] - lidar_input, imu_input
    # =========================================================
    
    # CARLA 토픽/프레임 → 실차 표준으로 변환
    #   /carla/hero/lidar (hero/lidar)  → /raw/points (os_sensor)
    #   /carla/hero/imu   (hero/imu)    → /raw/imu    (os_imu)
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

    # =========================================================
    # [CARLA 브리지 레이어] - 실차 배포 시 아래 노드 전체 제거
    # =========================================================
    # 실차 교체 목록:
    #   speed_imu_odom                                  → yhs_can_control (CAN 피드백 기반 /odom 발행)
    #   cmd_vel_to_ackermann + carla_ackermann_control  → yhs_can_control (CAN 제어)

    # CARLA speedometer + /raw/imu → /odom (실차에서는 yhs_can_control이 /odom 발행)
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
            ('/hero/imu',         '/raw/imu'),                  # bridge를 통해 변환된 토픽 구독
            ('/hero/wheel_odom',  '/odom'),                     # 실차 yhs_can_control과 동일한 토픽명
        ],
    )

    # CARLA 차량 제어: /cmd_vel → AckermannDrive → CARLA
    carla_ackermann_control = Node(
        package='carla_ackermann_control',
        executable='carla_ackermann_control_node',
        name='carla_ackermann_control',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('carla_ackermann_control'), 'config', 'settings.yaml'
            ]),
            {
                'role_name': 'hero',
                'control_loop_rate': 0.05,
            },
        ],
    )

    cmd_vel_to_ackermann = Node(
        package='my_pkg',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{
            'input_topic':         '/cmd_vel',
            'output_topic':        '/carla/hero/ackermann_cmd',
            'wheelbase':           0.856,
            'max_steering_angle':  0.463,
            'angular_deadband':    0.06,
            'min_speed_for_steer': 0.15,
            'max_speed':           0.6,
            'acceleration':        0.0,
            'jerk':                0.0,
        }],
    )

    return LaunchDescription(ARGUMENTS + [
        # 실차 공통
        lidar_tf,
        imu_tf,
        pointcloud_to_laserscan,
        nav2_bringup,
        rviz2,
        # 실차 전환 시 수정 필요
        sensor_bridge,
        # CARLA 브리지 (실차 배포 시 제거)
        speed_imu_odom,
        carla_ackermann_control,
        cmd_vel_to_ackermann,
    ])
