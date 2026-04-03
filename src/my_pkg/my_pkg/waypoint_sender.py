#!/usr/bin/env python3
"""
Waypoint sender: navigate_to_pose 액션으로 순차 주행
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


# waypoint 목록: (x, y, qz, qw) — /amcl_pose 기록값 사용
WAYPOINTS = [
    (46.626,  0.0,  -0.131, 0.991),   # WP1
    (48.180, -16.008, -0.892, 0.452),   # WP2
    (11.741, -17.762,  0.991, 0.131),   # WP3
    (20.609,  -0.391, -0.020, 1.000),   # WP4
]


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoints = WAYPOINTS
        self._index = 0

    def send_next(self):
        if self._index >= len(self._waypoints):
            self.get_logger().info('모든 waypoint 완료!')
            rclpy.shutdown()
            return

        x, y, qz, qw = self._waypoints[self._index]
        qx, qy = 0.0, 0.0

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'[{self._index + 1}/{len(self._waypoints)}] 목표 전송: ({x}, {y})'
        )
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal 거절됨')
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._index += 1
        self.send_next()


def main():
    rclpy.init()
    node = WaypointSender()
    node.get_logger().info('Waypoint sender 시작')
    node.send_next()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
