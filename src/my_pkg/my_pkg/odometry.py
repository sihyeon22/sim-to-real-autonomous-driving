#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SpeedImuOdomNode(Node):
    def __init__(self):
        super().__init__('speed_imu_odom_node')

        # Parameters
        self.declare_parameter('speed_topic', '/hero/speedometer')
        self.declare_parameter('imu_topic', '/hero/imu')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('odom_topic', '/hero/wheel_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('use_imu_angular_velocity', True)
        self.declare_parameter('invert_speed', False)

        self.speed_topic = self.get_parameter('speed_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.clock_topic = self.get_parameter('clock_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_imu_angular_velocity = self.get_parameter('use_imu_angular_velocity').value
        self.invert_speed = self.get_parameter('invert_speed').value

        # State
        self.speed_mps: float = 0.0
        self.current_yaw: Optional[float] = None
        self.initial_yaw: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.last_time: Optional[float] = None
        self.last_clock_msg = None

        self.x = 0.0
        self.y = 0.0

        self.has_speed = False
        self.has_imu = False

        self.last_imu_msg: Optional[Imu] = None

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32, self.speed_topic, self.speed_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.create_subscription(Clock, self.clock_topic, self.clock_cb, 100)

        self.get_logger().info('SpeedImuOdomNode started')
        self.get_logger().info(f'speed_topic: {self.speed_topic}')
        self.get_logger().info(f'imu_topic:   {self.imu_topic}')
        self.get_logger().info(f'clock_topic: {self.clock_topic}')
        self.get_logger().info(f'odom_topic:  {self.odom_topic}')
        self.get_logger().info(f'publish_tf:  {self.publish_tf}')

    def speed_cb(self, msg: Float32):
        speed = float(msg.data)
        if self.invert_speed:
            speed = -speed
        self.speed_mps = speed
        self.has_speed = True

    def imu_cb(self, msg: Imu):
        self.last_imu_msg = msg

        if not self.use_imu_angular_velocity:
            # 절대 yaw 모드 (CARLA ground truth orientation 사용)
            q = msg.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            if self.initial_yaw is None:
                self.initial_yaw = yaw
            self.current_yaw = normalize_angle(yaw - self.initial_yaw)
        else:
            # 상대 yaw 모드 (angular_velocity.z 적분, 실차 전환용)
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if self.current_yaw is None:
                self.current_yaw = 0.0
                self._last_imu_stamp = stamp
            else:
                dt = stamp - self._last_imu_stamp
                if 0.0 < dt < 0.5:
                    self.current_yaw = normalize_angle(
                        self.current_yaw + msg.angular_velocity.z * dt
                    )
                self._last_imu_stamp = stamp

        self.has_imu = True

    def clock_cb(self, msg: Clock):
        sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.last_clock_msg = msg

        if self.last_time is None:
            self.last_time = sim_time
            if self.current_yaw is not None:
                self.last_yaw = self.current_yaw
            return

        dt = sim_time - self.last_time
        if dt <= 0.0:
            return

        # 너무 긴 공백의 integration skip
        if dt > 0.5:
            self.last_time = sim_time
            if self.current_yaw is not None:
                self.last_yaw = self.current_yaw
            return

        self.last_time = sim_time

        if not self.has_speed or not self.has_imu or self.current_yaw is None:
            return

        yaw = self.current_yaw
        v = self.speed_mps

        # position integration
        self.x += v * math.cos(yaw) * dt
        self.y += v * math.sin(yaw) * dt

        # angular velocity
        omega = 0.0
        if self.use_imu_angular_velocity and self.last_imu_msg is not None:
            omega = float(self.last_imu_msg.angular_velocity.z)
        elif self.last_yaw is not None:
            dyaw = normalize_angle(yaw - self.last_yaw)
            omega = dyaw / dt

        self.last_yaw = yaw

        self.publish_odom(msg.clock, yaw, v, omega)

    def publish_odom(self, stamp, yaw: float, v: float, omega: float):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # 대충이라도 covariance 넣는 게 낫다
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0,0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0,0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0,0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02
        ]

        odom.twist.covariance = [
            0.02, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.2,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0,0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0,0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0,0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = yaw_to_quaternion(yaw)
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
