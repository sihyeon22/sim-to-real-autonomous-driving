#!/usr/bin/env python3

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = _normalize_quat(q)
    return (-x, -y, -z, w)


def _quat_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rotate_vec_by_quat_inv(
    q: Tuple[float, float, float, float],
    v: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    # v' = q^-1 * [v, 0] * q
    qi = _quat_inverse(q)
    vq = (v[0], v[1], v[2], 0.0)
    r = _quat_multiply(_quat_multiply(qi, vq), q)
    return (r[0], r[1], r[2])


def _yaw_from_quat(q: Tuple[float, float, float, float]) -> float:
    x, y, z, w = _normalize_quat(q)
    # Standard ZYX yaw extraction.
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OdomTfFromMapPose(Node):
    def __init__(self) -> None:
        super().__init__('odom_tf_from_map_pose')

        self.declare_parameter('input_topic', '/carla/hero/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'hero')

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, self._input_topic, self._on_odom, 10)

        self._have_t0 = False
        self._p0_xy = (0.0, 0.0)
        self._yaw0 = 0.0

        self.get_logger().info(
            f'Listening {self._input_topic} and publishing TF {self._odom_frame} -> {self._base_frame}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        q = _normalize_quat(q)

        yaw = _yaw_from_quat(q)

        if not self._have_t0:
            self._p0_xy = (p[0], p[1])
            self._yaw0 = yaw
            self._have_t0 = True
            self.get_logger().info('Captured initial pose T0 from odometry')

        # 2D relative transform from initial pose:
        # - Use only x, y, yaw
        # - Force z, roll, pitch to zero for 2D SLAM consistency
        dx = p[0] - self._p0_xy[0]
        dy = p[1] - self._p0_xy[1]

        c0 = math.cos(self._yaw0)
        s0 = math.sin(self._yaw0)
        x_rel = c0 * dx + s0 * dy
        y_rel = -s0 * dx + c0 * dy
        yaw_rel = _normalize_angle(yaw - self._yaw0)
        q_rel = _quat_from_yaw(yaw_rel)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame

        t.transform.translation.x = x_rel
        t.transform.translation.y = y_rel
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q_rel[0]
        t.transform.rotation.y = q_rel[1]
        t.transform.rotation.z = q_rel[2]
        t.transform.rotation.w = q_rel[3]

        self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfFromMapPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
