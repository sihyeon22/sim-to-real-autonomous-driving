#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive geometry_nav_msgs::Twist and publish carla_msgs::CarlaEgoVehicleControl

use max wheel steer angle
"""

import sys

import ros_compatibility as roscomp
from ros_compatibility.exceptions import ROSException
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo  # pylint: disable=import-error
from geometry_msgs.msg import Twist  # pylint: disable=import-error


class TwistToVehicleControl(CompatibleNode):  # pylint: disable=too-few-public-methods
    """
    receive geometry_nav_msgs::Twist and publish carla_msgs::CarlaEgoVehicleControl

    use max wheel steer angle
    """

    MAX_LON_ACCELERATION = 10

    def __init__(self):
        """
        Constructor
        """
        super(TwistToVehicleControl, self).__init__("twist_to_control")

        self.role_name = self.get_param("role_name", "hero")
        self.max_steering_angle = None
        # Tune Twist->vehicle-control conversion for low-speed Nav2 commands.
        # Nav2 often publishes linear.x around 0.3 m/s, which the original
        # mapping converted to throttle=0.03, too small to move the CARLA car.
        self.max_cmd_speed = float(self.get_param("max_cmd_speed", 1.0))
        self.min_forward_throttle = float(self.get_param("min_forward_throttle", 0.15))
        self.min_reverse_throttle = float(self.get_param("min_reverse_throttle", 0.15))
        self.max_cmd_yaw_rate = float(self.get_param("max_cmd_yaw_rate", 0.5))
        self.steer_gain = float(self.get_param("steer_gain", 1.0))

        self.new_subscription(
            CarlaEgoVehicleInfo,
            "/carla/{}/vehicle_info".format(self.role_name),
            self.update_vehicle_info,
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.new_subscription(
            Twist,
            "/carla/{}/twist".format(self.role_name),
            self.twist_received,
            qos_profile=10)

        self.pub = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(self.role_name),
            qos_profile=10)

    def update_vehicle_info(self, vehicle_info):
        """
        callback to receive ego-vehicle info
        """
        if not vehicle_info.wheels:  # pylint: disable=no-member
            self.logerr("Cannot determine max steering angle: Vehicle has no wheels.")
            sys.exit(1)

        self.max_steering_angle = vehicle_info.wheels[0].max_steer_angle  # pylint: disable=no-member
        if not self.max_steering_angle:
            self.logerr("Cannot determine max steering angle: Value is %s",
                        self.max_steering_angle)
            sys.exit(1)
        self.loginfo("Vehicle info received. Max steering angle={}".format(self.max_steering_angle))

    def twist_received(self, twist):
        """
        receive twist and convert to carla vehicle control
        """
        if self.max_steering_angle is None:
            self.logwarn("Did not yet receive vehicle info.")
            return

        control = CarlaEgoVehicleControl()
        if twist == Twist():
            # stop
            control.throttle = 0.
            control.brake = 1.
            control.steer = 0.
        else:
            if twist.linear.x > 0:
                control.throttle = min(1.0, twist.linear.x / max(self.max_cmd_speed, 1e-6))
                control.throttle = max(self.min_forward_throttle, control.throttle)
            else:
                control.reverse = True
                control.throttle = min(1.0, abs(twist.linear.x) / max(self.max_cmd_speed, 1e-6))
                control.throttle = max(self.min_reverse_throttle, control.throttle)

            steer = -(twist.angular.z / max(self.max_cmd_yaw_rate, 1e-6)) * self.steer_gain
            control.steer = max(-1.0, min(1.0, steer))
        try:
            self.pub.publish(control)
        except ROSException as e:
            if roscomp.ok():
                self.logwarn("Error while publishing control: {}".format(e))


def main(args=None):
    """
    main function

    :return:
    """
    roscomp.init("twist_to_control", args)

    twist_to_vehicle_control = None
    try:
        twist_to_vehicle_control = TwistToVehicleControl()
        twist_to_vehicle_control.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if twist_to_vehicle_control is not None:
            twist_to_vehicle_control.loginfo("Done, deleting twist to control")
            del twist_to_vehicle_control
        roscomp.shutdown()


if __name__ == "__main__":
    main()
