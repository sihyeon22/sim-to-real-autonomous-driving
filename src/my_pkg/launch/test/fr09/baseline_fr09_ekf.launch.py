# File: baseline_fr09_ekf.launch.py
# Author: sihyeon
# Date: 2026.04.15 WED
# Description:
#     - speedometer + IMU dead reckoning 기반 /odom_wheel 생성
#     - robot_localization EKF로 /odom_wheel(vx, vyaw) + IMU(ax, ay) 융합
#     - EKF 출력 /odometry/filtered를 Nav2 odometry로 연결

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
            EnvironmentVariable('HOME'), 'CARLA_ws', 'maps', 'parking', 'mk_best.yaml'
        ]),
        description='Full path to saved map yaml',
    ),
    DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_pkg'), 'config', 'nav2_params_fr09_ekf.yaml'
        ]),
        description='Full path to Nav2 params yaml for fr09 baseline',
    ),
    DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with Nav2 default config',
    ),
    DeclareLaunchArgument(
        'use_cmd_vel_relay',
        default_value='true',
        description='Convert /cmd_vel to /carla/hero/ackermann_cmd via Ackermann control',
    ),
]


def generate_launch_description():
    # LaunchConfiguration 변수
    map_yaml          = LaunchConfiguration('map_yaml')
    nav2_params       = LaunchConfiguration('nav2_params')
    use_rviz          = LaunchConfiguration('use_rviz')
    use_cmd_vel_relay = LaunchConfiguration('use_cmd_vel_relay')


    # TF: base_link -> 센서 프레임
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hero_to_lidar_static_tf',
        arguments=[
            '--x', '0.456', '--y', '0', '--z', '1.135',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'hero/lidar',
        ],
        output='screen',
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hero_to_imu_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.5',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'hero/imu',
        ],
        output='screen',
    )


    # Odometry: speedometer + IMU dead reckoning
    # publish_tf를 False로 설정 -> odom->base_link TF는 EKF 노드가 발행
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


    # EKF: /odom_wheel(vx, vyaw) + IMU(ax, ay) 융합
    # 출력: /odometry/filtered, odom->base_link TF 발행
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('my_pkg'), 'config', 'ekf.yaml']),
            {'use_sim_time': True},
        ],
    )


    # 센서: PointCloud -> LaserScan 변환
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
            'min_height': -0.95,
            'max_height': 0.00,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.0087,
            'range_min': 0.19,
            'range_max': 12.0,
        }],
        remappings=[
            ('cloud_in', '/carla/hero/lidar'),
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
            'map':             map_yaml,
            'params_file':     nav2_params,
            'use_sim_time':    'True',
            'autostart':       'True',
            'use_composition': 'False',
        }.items(),
    )


    # 차량 제어: cmd_vel -> Ackermann -> CARLA
    carla_ackermann_control = Node(
        condition=IfCondition(use_cmd_vel_relay),
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
        condition=IfCondition(use_cmd_vel_relay),
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


    return LaunchDescription(ARGUMENTS + [
        lidar_tf,                   # TF
        imu_tf,
        speed_imu_odom,             # odometry
        ekf_node,                   # EKF 융합
        pointcloud_to_laserscan,    # 센서
        nav2_bringup,               # Nav2
        carla_ackermann_control,    # 제어
        cmd_vel_to_ackermann,
        rviz2,                      # 시각화
    ])
