#!/usr/bin/env python3

import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class CmdVelToAckermann(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_ackermann')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/carla/hero/ackermann_cmd')
        self.declare_parameter('wheelbase', 3.0)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('angular_deadband', 0.03)
        self.declare_parameter('min_speed_for_steer', 0.2)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('acceleration', 1.5)
        self.declare_parameter('jerk', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.min_speed_for_steer = float(self.get_parameter('min_speed_for_steer').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.acceleration = float(self.get_parameter('acceleration').value)
        self.jerk = float(self.get_parameter('jerk').value)

        self._publisher = self.create_publisher(AckermannDrive, output_topic, 10)
        self._subscription = self.create_subscription(Twist, input_topic, self._on_cmd_vel, 10)

        self.get_logger().info(
            f'Converting {input_topic} -> {output_topic} '
            f'(wheelbase={self.wheelbase}, max_steering_angle={self.max_steering_angle})'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        drive = AckermannDrive()
        drive.speed = clamp(linear_x, -self.max_speed, self.max_speed)
        drive.acceleration = self.acceleration
        drive.jerk = self.jerk
        drive.steering_angle_velocity = 0.0

        if abs(linear_x) < self.min_speed_for_steer or abs(angular_z) < self.angular_deadband:
            drive.steering_angle = 0.0
        else:
            steering_angle = math.atan(self.wheelbase * angular_z / linear_x)
            drive.steering_angle = clamp(
                steering_angle,
                -self.max_steering_angle,
                self.max_steering_angle,
            )

        self._publisher.publish(drive)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
