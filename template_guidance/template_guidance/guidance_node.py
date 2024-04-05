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
import geometry_msgs.msg

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from template_guidance.straight_line import straight_line, update_law
from template_guidance.stationkeeping import stationkeeping
from template_guidance.path import path, HybridPathGenerator, HybridPathSignals, update_s

class Guidance(rclpy.node.Node):
    def __init__(self):
        super().__init__("cse_thrust_allocation")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.pubs["reference"] = self.create_publisher(
            tmr4243_interfaces.msg.Reference, '/CSEI/control/reference', 1)

        self.eta_pub = self.create_publisher(
            geometry_msgs.msg.Pose, 'eta', 1)
        
        self.current_guidance = self.declare_parameter('task', 'stationkeeping')
        # self.current_guidance = self.declare_parameter('guidance', 'straight_line')
        # self.current_guidance = self.declare_parameter('guidance', 'hybridpath')

        self.last_transform = None
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        guidance_period = 0.1 # seconds
        self.guidance_timer = self.create_timer(guidance_period, self.guidance_callback)
        # 4 corner test waypoints
        waypoints = np.array([[0,0],[30, 0], [30, 30], [0, 30], [0, 0]])
        r = 1
        lambda_val = 0.3
        generator = HybridPathGenerator(waypoints, r, lambda_val)
        self.path = generator.Path
        self.u_desired = 2.5
        self.s = 0

    def timer_callback(self):

        try:
            self.last_transform = self.tf_buffer.lookup_transform(
                "base_link",
                "world",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform : {ex}')

        self.guidance = self.get_parameter('task')


        self.get_logger().info(f"Parameter task: {self.guidance.value}", throttle_duration_sec=1.0)


    def guidance_callback(self):

        self.current_guidance = self.get_parameter('task')

        if "stationkeeping" in self.current_guidance.value:
            eta_d, eta_ds, eta_ds2 = stationkeeping()

            n = tmr4243_interfaces.msg.Reference()
            n.eta_d = eta_d
            n.eta_ds = eta_ds
            n.eta_ds2 = eta_ds2
            self.pubs["reference"].publish(n)

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
            dt = 0.1
            self.s += update_s(self.path, dt, self.u_desired, self.s)
            signals = HybridPathSignals(self.path, self.s)
            n = tmr4243_interfaces.msg.Reference()
            pos = signals.pd
            pos_der = signals.pd_der[0]
            pos_dder = signals.pd_der[1]
            psi, psi_der, psi_dder = signals.get_heading()
            n.eta_d = np.array([pos[0], pos[1], psi]).tolist()
            eta_d_msg = geometry_msgs.msg.Pose()
            eta_d_msg.position.x = pos[0]
            eta_d_msg.position.y = pos[1]
            eta_d_msg.orientation.z = psi
            self.eta_pub.publish(eta_d_msg)

            n.eta_ds = np.array([pos_der[0], pos_der[1], psi_der]).tolist()
            n.eta_ds2 = np.array([pos_dder[0], pos_dder[1], psi_dder]).tolist()
            self.pubs["reference"].publish(n)

            v = tmr4243_interfaces.msg.Reference()
            v.w = 0.0
            v.v_s, v.v_ss = signals.calc_vs(self.u_desired)
            self.pubs["reference"].publish(v)



def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = Guidance()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

