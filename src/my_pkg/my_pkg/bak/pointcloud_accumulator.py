#!/usr/bin/env python3
"""
PointCloud2 프레임 누적 노드

- 최근 N 프레임의 포인트클라우드를 버퍼에 쌓아서 합쳐 publish
- 32채널 LiDAR의 sparsity 문제를 완화
- 연석처럼 매 프레임 소수 빔만 걸리는 낮은 장애물 검출에 효과적
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')

        self.declare_parameter('buffer_size',   5)
        self.declare_parameter('input_topic',  '/carla/hero/lidar')
        self.declare_parameter('output_topic', '/lidar/accumulated')

        self.buffer_size  = self.get_parameter('buffer_size').value
        in_topic          = self.get_parameter('input_topic').value
        out_topic         = self.get_parameter('output_topic').value

        self.buffer = deque(maxlen=self.buffer_size)

        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, out_topic, 10)

        self.get_logger().info(
            f'PointCloudAccumulator  buffer={self.buffer_size}  '
            f'{in_topic} → {out_topic}')

    def callback(self, msg: PointCloud2):
        # 현재 프레임 포인트 읽기
        pts = pc2.read_points_numpy(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)

        if pts.shape[0] == 0:
            return

        self.buffer.append(pts.astype(np.float32))

        # 버퍼 내 전체 프레임 합치기
        merged = np.concatenate(list(self.buffer), axis=0)

        # 최신 프레임의 헤더를 그대로 사용 (timestamp, frame_id 유지)
        out_msg = self._make_pc2(msg.header, merged)
        self.pub.publish(out_msg)

    @staticmethod
    def _make_pc2(header, points: np.ndarray) -> PointCloud2:
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        itemsize = 12
        msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=itemsize,
            row_step=itemsize * len(points),
            data=points.astype(np.float32).tobytes(),
        )
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
