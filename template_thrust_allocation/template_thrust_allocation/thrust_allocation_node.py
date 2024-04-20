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
# Maintainer: Emir Cem Gezer, Petter Hangerhageen
# Email: emir.cem.gezer@ntnu.no, emircem.gezer@gmail.com, me@emircem.com, petthang@stud.ntnu.no
# Year: 2024
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import rclpy
import rclpy.node
import std_msgs.msg
import geometry_msgs.msg
import numpy as np
import time

from template_thrust_allocation.thruster_allocation import thruster_allocation, thrust_allocation_two_thrusters

class ThrustAllocation(rclpy.node.Node):
    def __init__(self):
        super().__init__("tmr4243_thrust_allocation_node")

        self.last_received_forces = None

        self.pubs = {}
        self.subs = {}

        self.subs["tau_cmd"] = self.create_subscription(
            geometry_msgs.msg.Wrench, '/CSEI/control/tau_cmd', self.tau_cmd_callback, 1)

        self.pubs["u_cmd"] = self.create_publisher(
            std_msgs.msg.Float32MultiArray, '/CSEI/control/u_cmd', 1)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0

        self.something = False

    def timer_callback(self):

        if self.last_received_forces is not None:
            tau = np.array([self.last_received_forces.force.x, self.last_received_forces.force.y, self.last_received_forces.torque.z])
            u = thruster_allocation(tau)
            #u = thrust_allocation_two_thrusters(tau)

            u_cmd_msg = std_msgs.msg.Float32MultiArray()
            u_cmd_msg.data = u
            self.pubs["u_cmd"].publish(u_cmd_msg)

            #self.get_logger().info(f"Published u_cmd: {u}")

            #self.last_recived_forces = None

    def tau_cmd_callback(self, msg):
        self.last_received_forces = msg


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = ThrustAllocation()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()