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
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com
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

from pyquaternion import Quaternion as quat

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
            std_msgs.msg.Float32MultiArray, '/CSEI/state/eta', 1)
        self.pubs["tau"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/tau', 1)

        self.subs["u_cmd"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/control/u_cmd', self.u_command_callback, 10)

        self.tau = np.zeros(3)

        self.state_loop = self.create_timer(0.1, self.state_timer_callback)

    def state_timer_callback(self):

        try:
            pose = self.tf_buffer.lookup_transform(
                "world",
                "base_link",
                rclpy.time.Time())
            eta = [None] * 3

            eta[0] = pose.transform.translation.x
            eta[1] = pose.transform.translation.y

            eta[2], _, _ = quat(
                pose.transform.rotation.w,
                pose.transform.rotation.x,
                pose.transform.rotation.y,
                pose.transform.rotation.z
            ).yaw_pitch_roll

            msg = std_msgs.msg.Float32MultiArray()

            msg.data = eta
            self.pubs['eta'].publish(msg)
            #self.get_logger().info(f"eta: {eta}")

        except TransformException as ex:
            self.get_logger().info(f'Could not transform : {ex}', throttle_duration_sec=1.0)

        msg = std_msgs.msg.Float32MultiArray()
        msg.data = self.tau.tolist()
        self.pubs['tau'].publish(msg)
        #self.get_logger().info(f"tau: {self.tau}")

    def u_command_callback(self, msg: std_msgs.msg.Float32MultiArray):
        # Check if the message is of correct size
        if len(msg.data) != 5:
            self.get_logger().warn(
                f"ill sized u_cmd array with length {len(msg.data)}: {msg.data}")
            self.get_logger().info(f"{msg}")
            self.tau = np.zeros(3)
            return

        # Extract the data
        u = list(msg.data)
        #self.get_logger().info(f"u_cmd: {u}")
        u0 = u[0]
        u1 = u[1]
        u2 = u[2]
        a1 = u[3]
        a2 = u[4]

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

        B = np.array([
            [0, np.cos(a1), np.cos(a2)],
            [1, np.sin(a1), np.sin(a2)],
            [0.3875,
             0.0550 * np.cos(a1) - 0.4574 * np.sin(a1),
             -0.4574 * np.sin(a2) - 0.0550 * np.cos(a2)
            ]
        ])
        self.tau = B @ np.array([
            [np.clip(u0, -1, 1)],
            [np.clip(u1, -2, 2)],
            [np.clip(u2, -2, 2)]
        ]).flatten()
        #self.get_logger().info(f"tau: {self.tau}")



def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = UtilityNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
