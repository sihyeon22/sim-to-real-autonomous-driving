#!/usr/bin/env python3

import math

import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class CmdVelToVehicleControl(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_vehicle_control')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/carla/hero/vehicle_control_cmd')
        self.declare_parameter('manual_override_topic', '/carla/hero/vehicle_control_manual_override')
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('max_throttle', 0.35)
        self.declare_parameter('max_steer', 0.35)
        self.declare_parameter('steer_gain', 0.6)
        self.declare_parameter('brake_on_stop', 0.3)
        self.declare_parameter('stop_speed_threshold', 0.05)
        self.declare_parameter('min_drive_throttle', 0.18)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        manual_override_topic = self.get_parameter('manual_override_topic').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.steer_gain = float(self.get_parameter('steer_gain').value)
        self.brake_on_stop = float(self.get_parameter('brake_on_stop').value)
        self.stop_speed_threshold = float(self.get_parameter('stop_speed_threshold').value)
        self.min_drive_throttle = float(self.get_parameter('min_drive_throttle').value)

        self._control_publisher = self.create_publisher(CarlaEgoVehicleControl, output_topic, 10)
        self._override_publisher = self.create_publisher(Bool, manual_override_topic, 10)
        self._subscription = self.create_subscription(Twist, input_topic, self._on_cmd_vel, 10)

        self.get_logger().info(
            f'Converting {input_topic} -> {output_topic} '
            f'(max_speed={self.max_speed}, max_throttle={self.max_throttle}, max_steer={self.max_steer})'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Ensure bridge uses the normal vehicle_control_cmd path.
        self._override_publisher.publish(Bool(data=False))

        control = CarlaEgoVehicleControl()
        control.header.stamp = self.get_clock().now().to_msg()
        control.header.frame_id = 'hero'

        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        if abs(linear_x) <= self.stop_speed_threshold:
            control.throttle = 0.0
            control.brake = self.brake_on_stop
            control.steer = 0.0
            control.reverse = False
        else:
            normalized_speed = clamp(abs(linear_x) / max(self.max_speed, 1e-6), 0.0, 1.0)
            throttle = max(self.min_drive_throttle, normalized_speed * self.max_throttle)
            control.throttle = clamp(throttle, 0.0, self.max_throttle)
            control.brake = 0.0
            control.reverse = linear_x < 0.0

            steer = self.steer_gain * angular_z
            if linear_x < 0.0:
                steer *= -1.0
            control.steer = clamp(steer, -self.max_steer, self.max_steer)

        control.hand_brake = False
        control.manual_gear_shift = False
        control.gear = 0
        self._control_publisher.publish(control)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToVehicleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
