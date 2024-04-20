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

import numpy as np
import rclpy
import rclpy.node
import tmr4243_interfaces.msg
import std_msgs.msg
from geometry_msgs.msg import Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from template_guidance.straight_line import straight_line, update_law
from template_guidance.stationkeeping import ReferenceFilter
from template_guidance.path import HybridPathGenerator, HybridPathSignals
from tmr4243_interfaces.srv import Waypoint
from tmr4243_interfaces.msg import Reference

class Guidance(rclpy.node.Node):
    def __init__(self):
        super().__init__("cse_thrust_allocation")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.waypoint_server = self.create_service(Waypoint, 'waypoint_list', self.waypoint_callback)

        # ros2 service call waypoint_list tmr4243_interfaces/srv/Waypoint "waypoint: [{x: 0.0, y: 0.0, z: 0.0}, {x: 5.0, y: 5.0, z: 0.0}, {x: 10.0, y: 10.0, z: 0.0}, {x: 0.0, y: 10.0, z: 0.0}, {x: 0.0, y: 0.0, z: 0.0}]"
        # ros2 service call waypoint_list tmr4243_interfaces/srv/Waypoint "waypoint: [{x: 0.0, y: 0.0, z: 0.0}, {x: 10.0, y: 0.0, z: 0.0}]"

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False
        self.last_waypoint_reached = False

        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/eta', self.eta_callback, 1)

        self.pubs["reference"] = self.create_publisher(
            Reference, '/CSEI/control/reference', 1)
        
        self.pubs["DPref"] = self.create_publisher(
            Reference, '/CSEI/control/DPref', 1)
        
        self.current_guidance = self.declare_parameter('task', 'hybridpath')
        # self.current_guidance = self.declare_parameter('guidance', 'straight_line')
        # self.current_guidance = self.declare_parameter('guidance', 'hybridpath')

        self.last_transform = None
        self.eta = None
        
        self.r = 1
        self.lambda_val = 0.2
        self.mu = 0.03

        self.has_eta = False

        self.timer = self.create_timer(0.1, self.guidance_callback)

        self.reference_filter = ReferenceFilter()
        self.xd = np.zeros(9)

        self.init_pos = False
        

    def eta_callback(self, msg):
        self.eta = list(msg.data)
        self.has_eta = True

    def waypoint_callback(self, request, response):
        self.get_logger().info('Received waypoints, generating path...')
        self.generator = None
        self.path = None
        self.u_desired = 0.25
        self.waypoints = request.waypoint
        # self.get_logger().info(f'Waypoints: {self.waypoints}')
        self.generator = HybridPathGenerator(self.waypoints, self.r, self.lambda_val)
        # self.get_logger().info(f'Path generating: {self.generator.WP}')
        self.path = self.generator.path
        self.waypoints_received = True
        self.waiting_message_printed = False  # Reset this flag to handle multiple waypoint sets
        self.s = 0
        signals = HybridPathSignals(self.path, self.s)
        self.w = signals.get_w(self.mu, self.eta)
        response.success = True
        return response

    def guidance_callback(self):

        self.current_guidance = self.get_parameter('task')

        if self.last_waypoint_reached and self.has_eta:
            last_waypoint = self.waypoints[-1]
            eta_ref = np.array([last_waypoint.x, last_waypoint.y, np.pi])
            x_next = self.reference_filter.step(eta_ref, self.xd)
            self.xd = x_next
            ref_msg = Reference()
            ref_msg.eta_d = x_next[0:3].tolist()
            ref_msg.eta_ds = x_next[3:6].tolist()
            self.pubs["DPref"].publish(ref_msg)

        elif "stationkeeping" in self.current_guidance.value:
            # self.get_logger().info('Stationkeeping')
            if self.waypoints_received and self.has_eta:
                if not self.init_pos:
                    self.xd[0:3] = self.eta
                    self.init_pos = True
                last_waypoint = self.waypoints[-1]
                eta_ref = np.array([last_waypoint.x, last_waypoint.y, np.pi])
                x_next = self.reference_filter.step(eta_ref, self.xd)
                self.xd = x_next
                ref_msg = Reference()
                ref_msg.eta_d = x_next[0:3].tolist()
                ref_msg.eta_ds = x_next[3:6].tolist()
                self.pubs["DPref"].publish(ref_msg)

            else:
                if not self.waiting_message_printed:
                    self.get_logger().info('Waiting for waypoints to be received')
                    self.waiting_message_printed = True

        elif "straight_line" in self.current_guidance.value:
            eta_d, eta_ds, eta_ds2 = straight_line()
            w, v_s, v_ss = update_law()
            n = tmr4243_interfaces.msg.Reference()
            n.eta_d = eta_d
            n.eta_ds = eta_ds
            n.eta_ds2 = eta_ds2
            self.pubs["reference"].publish(n)

            v = tmr4243_interfaces.msg.Reference()
            v.w = w
            v.v_s = v_s
            v.v_ss = v_ss
            self.pubs["reference"].publish(v)
        
        elif "hybridpath" in self.current_guidance.value:
            if self.waypoints_received and self.has_eta and not self.last_waypoint_reached:
                dt = 0.1
                self.s = self.generator.update_s(self.path, dt, self.u_desired, self.s, self.w)
                signals = HybridPathSignals(self.path, self.s)
                self.w = signals.get_w(self.mu, self.eta)

                n = tmr4243_interfaces.msg.Reference()
                pos = signals.get_position()
                pos_der = signals.get_derivatives()[0]
                pos_dder = signals.get_derivatives()[1]

                psi = 0*signals.get_heading()
                psi_der = 0*signals.get_heading_derivative()
                psi_dder = 0*signals.get_heading_second_derivative()

                n.eta_d = np.array([pos[0], pos[1], psi], dtype=float).tolist()
                n.eta_ds = np.array([pos_der[0], pos_der[1], psi_der], dtype=float).tolist()
                n.eta_ds2 = np.array([pos_dder[0], pos_dder[1], psi_dder], dtype=float).tolist()

                n.w = signals.get_w(self.mu, self.eta)
                n.v_s = signals.get_vs(self.u_desired)
                n.v_ss = signals.get_vs_derivative(self.u_desired)
                self.pubs["reference"].publish(n)

                if self.s >= self.path.NumSubpaths-0.6 and self.last_waypoint_reached == False:
                    # self.waypoints_received = False
                    self.waiting_message_printed = False
                    self.get_logger().info('Closing in on the last waypoint, switching to DP')
                    self.last_waypoint_reached = True
                    self.xd[0:3] = n.eta_d
                    self.xd[3:6] = np.array([0.15, 0.15, 0])

            else:
                if not self.waiting_message_printed:
                    self.get_logger().info('Waiting for waypoints to be received')
                    self.waiting_message_printed = True



def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

