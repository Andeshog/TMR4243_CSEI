#!/usr/bin/env python3
#
# This file is part of CyberShip Enterpries Suite.
#
# CyberShip Enterpries Suite software is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CyberShip Enterpries Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# CyberShip Enterpries Suite. If not, see <https://www.gnu.org/licenses/>.
#
# Maintainer: Emir Cem Gezer
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import math
import numpy as np

import rclpy
import rclpy.node

import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler


class UtilityNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('tmr4243_utility')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.pubs["tunnel"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/tunnel/command', 1)
        self.pubs["port"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/port/command', 1)
        self.pubs["starboard"] = self.create_publisher(
            geometry_msgs.msg.Wrench, '/CSEI/thrusters/starboard/command', 1)
        self.pubs["eta"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/eta', 1
        )
        self.pubs["tau"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/tau', 1)

        self.subs["allocated"] = self.create_subscription(
            geometry_msgs.msg.Wrench, '/CSEI/allocated', self.allocated_callback, 10)

        self.subs["joy"] = self.create_subscription(
            sensor_msgs.msg.Joy, '/joy', self.joy_callback, 10)

        self.subs["u_cmd"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/control/u_cmd', self.u_command_callback, 10)

        self.joystick_name = self.declare_parameter('joystick_name', 'PS4')

        self.current_task = self.declare_parameter('task', 'simple')
        self.current_task.value

        self.last_transform = None
        self.eta_publisher = self.create_timer(0.1, self.eta_publisher)

        # self.tau = np.zeros((3), dtype=float)
        # self.tau_publisher = self.create_timer(0.1, self.tau_publisher)

    def eta_publisher(self):

        try:
            self.last_transform = self.tf_buffer.lookup_transform(
                "world",
                "world",
                "base_link",
                rclpy.time.Time())
            eta = [None] * 3

            eta[0] = self.last_transform.transform.translation.x #*(-1.0)
            eta[1] = self.last_transform.transform.translation.y
            rotation_list = [self.last_transform.transform.rotation.w,
                                self.last_transform.transform.rotation.x,
                                self.last_transform.transform.rotation.y,
                                self.last_transform.transform.rotation.z]
            roll, pitch, yaw = quat2euler(rotation_list)
            # [roll, pitch, yaw] = self.quat2eul(
            #     self.last_transform.transform.rotation.x,
            #     self.last_transform.transform.rotation.y  ,
            #     self.last_transform.transform.rotation.z,
            #     self.last_transform.transform.rotation.w
            # )
            eta[2] = yaw #* (-1.0)

            msg = std_msgs.msg.Float32MultiArray()
            msg.data = eta
            self.pubs['eta'].publish(msg)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform : {ex}')

    def u_command_callback(self, msg: std_msgs.msg.Float32MultiArray):
        # Check if the message is of correct size
        if len(msg.data) != 5:
            self.get_logger().warn(
                f"ill sized u_cmd array with length {len(msg.data)}: {msg.data}")
            self.get_logger().info(f"{msg}")
            return

        # Extract the data
        u0, u1, u2, a1, a2 = msg.data

        # Create the messages and publish
        f = geometry_msgs.msg.Wrench()
        f.force.x = u0
        self.pubs['tunnel'].publish(f)

        f = geometry_msgs.msg.Wrench()
        f.force.x = u1 * math.cos(a1)
        f.force.y = u1 * math.sin(a1)
        self.pubs['port'].publish(f)

        f = geometry_msgs.msg.Wrench()
        f.force.x = u2 * math.cos(a2)
        f.force.y = u2 * math.sin(a2)
        self.pubs['starboard'].publish(f)

    @staticmethod
    def quat2eul(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def joy_callback(self, msg: sensor_msgs.msg.Joy):
        tau = [None] * 3

        tau[0] = msg.axes[1]
        tau[1] = msg.axes[0]
        tau[2] = msg.axes[3]

        msg = std_msgs.msg.Float32MultiArray()
        msg.data = tau
        self.pubs['tau'].publish(msg)

    def tau_publisher(self):
        msg = std_msgs.msg.Float32MultiArray()

        msg.data = [self.tau[0], -self.tau[1], -self.tau[2]]
        self.pubs['tau'].publish(msg)


    def allocated_callback(self, msg: geometry_msgs.msg.Wrench):
        tau = [None] * 3

        tau[0] = msg.force.x
        tau[1] = msg.force.y
        tau[2] = msg.torque.z

        msg = std_msgs.msg.Float32MultiArray()
        msg.data = tau
        self.pubs['tau'].publish(msg)

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = UtilityNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()