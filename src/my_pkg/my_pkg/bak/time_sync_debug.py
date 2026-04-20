#!/usr/bin/env python3

from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TimeSyncDebug(Node):
    def __init__(self) -> None:
        super().__init__('time_sync_debug')

        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('odom_topic', '/carla/hero/odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('report_rate', 2.0)

        self._clock_topic = self.get_parameter('clock_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._scan_topic = self.get_parameter('scan_topic').value
        self._report_rate = float(self.get_parameter('report_rate').value)

        self._clock_sec: Optional[float] = None
        self._odom_sec: Optional[float] = None
        self._scan_sec: Optional[float] = None
        self._odom_frame: Optional[str] = None
        self._scan_frame: Optional[str] = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Clock, self._clock_topic, self._on_clock, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, sensor_qos)
        self.create_timer(1.0 / max(self._report_rate, 0.1), self._report)

        self.get_logger().info(
            f'Monitoring clock={self._clock_topic}, odom={self._odom_topic}, '
            f'scan={self._scan_topic} at {self._report_rate:.1f} Hz'
        )

    def _on_clock(self, msg: Clock) -> None:
        self._clock_sec = stamp_to_sec(msg.clock)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_sec = stamp_to_sec(msg.header.stamp)
        self._odom_frame = f'{msg.header.frame_id}->{msg.child_frame_id}'

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_sec = stamp_to_sec(msg.header.stamp)
        self._scan_frame = msg.header.frame_id

    def _fmt_delta(self, value: Optional[float]) -> str:
        if value is None:
            return 'n/a'
        return f'{value:+.3f}s'

    def _report(self) -> None:
        if self._clock_sec is None and self._odom_sec is None and self._scan_sec is None:
            return

        odom_minus_clock = None if self._clock_sec is None or self._odom_sec is None else self._odom_sec - self._clock_sec
        scan_minus_clock = None if self._clock_sec is None or self._scan_sec is None else self._scan_sec - self._clock_sec
        scan_minus_odom = None if self._scan_sec is None or self._odom_sec is None else self._scan_sec - self._odom_sec

        self.get_logger().info(
            'clock={clock} odom={odom} ({odom_frame}) scan={scan} ({scan_frame}) '
            'd(odom-clock)={d_oc} d(scan-clock)={d_sc} d(scan-odom)={d_so}'.format(
                clock='n/a' if self._clock_sec is None else f'{self._clock_sec:.3f}',
                odom='n/a' if self._odom_sec is None else f'{self._odom_sec:.3f}',
                odom_frame=self._odom_frame or 'n/a',
                scan='n/a' if self._scan_sec is None else f'{self._scan_sec:.3f}',
                scan_frame=self._scan_frame or 'n/a',
                d_oc=self._fmt_delta(odom_minus_clock),
                d_sc=self._fmt_delta(scan_minus_clock),
                d_so=self._fmt_delta(scan_minus_odom),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimeSyncDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
