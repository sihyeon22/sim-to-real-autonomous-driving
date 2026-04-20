#!/usr/bin/env python3
# File: sensor_bridge.py
# Description:
#     센서 토픽 -> 공용 토픽 전환 브리지 노드
#
#     변환 내용:
#       토픽: /carla/hero/lidar -> /raw/points  (frame_id: hero/lidar -> os_sensor)
#       토픽: /carla/hero/imu   -> /raw/imu     (frame_id: hero/imu  -> os_imu)
#
#     실차 배포 시 실차에서 발행되는 토픽명으로 변경

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')

        # 파라미터 선언 (launch 파일에서 덮어쓸 수 있음)
        self.declare_parameter('lidar_input',  '/carla/hero/lidar')
        self.declare_parameter('lidar_output', '/raw/points')
        self.declare_parameter('lidar_frame',  'os_sensor')

        self.declare_parameter('imu_input',  '/carla/hero/imu')
        self.declare_parameter('imu_output', '/raw/imu')
        self.declare_parameter('imu_frame',  'os_imu')

        lidar_in   = self.get_parameter('lidar_input').value
        lidar_out  = self.get_parameter('lidar_output').value
        self.lidar_frame = self.get_parameter('lidar_frame').value

        imu_in   = self.get_parameter('imu_input').value
        imu_out  = self.get_parameter('imu_output').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # Publisher / Subscriber
        self.lidar_pub = self.create_publisher(PointCloud2, lidar_out, 10)
        self.imu_pub   = self.create_publisher(Imu,         imu_out,   50)

        self.create_subscription(PointCloud2, lidar_in, self.lidar_cb, 10)
        self.create_subscription(Imu,         imu_in,   self.imu_cb,   50)

        self.get_logger().info('SensorBridge started')
        self.get_logger().info(f'  LiDAR: {lidar_in} -> {lidar_out} (frame: {self.lidar_frame})')
        self.get_logger().info(f'  IMU  : {imu_in}   -> {imu_out}   (frame: {self.imu_frame})')

    def lidar_cb(self, msg: PointCloud2):
        # frame_id를 CARLA 프레임(hero/lidar)에서 Ouster 표준 프레임(os_sensor)으로 교체
        msg.header.frame_id = self.lidar_frame
        self.lidar_pub.publish(msg)

    def imu_cb(self, msg: Imu):
        # frame_id를 CARLA 프레임(hero/imu)에서 Ouster 표준 프레임(os_imu)으로 교체
        msg.header.frame_id = self.imu_frame
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
