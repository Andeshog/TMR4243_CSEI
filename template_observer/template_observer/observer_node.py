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
# Year: 2022
# Copyright (C) 2024 NTNU Marine Cybernetics Laboratory

import rclpy
import rclpy.node
import numpy as np

import std_msgs.msg
from nav_msgs.msg import Odometry
import tmr4243_interfaces.msg

import std_srvs.srv
from template_observer.luenberg import Observer
from template_observer.wrap import wrap


class ObserverNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('cse_observer')

        self.L1 = self.declare_parameter('L1', [1.0] * 3)
        self.L2 = self.declare_parameter('L2', [1.0] * 3)
        self.L3 = self.declare_parameter('L3', [0.1] * 3)

        L1 = self.get_parameter('L1').get_parameter_value().double_array_value
        L2 = self.get_parameter('L2').get_parameter_value().double_array_value
        L3 = self.get_parameter('L3').get_parameter_value().double_array_value

        self.subs = {}
        self.pubs = {}


        self.subs["tau"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/tau', self.tau_callback, 10
        )
        
        self.subs["eta"] = self.create_subscription(
            std_msgs.msg.Float32MultiArray, '/CSEI/state/eta', self.eta_callback, 10
        )

        # self.subs["eta"] = self.create_subscription(
        #     Odometry, '/CSEI/odom', self.odom_callback, 10
        # )

        # self.pubs['psi'] = self.create_publisher(
        #     std_msgs.msg.Float32, '/CSEI/state/psi', 1
        # )

        self.pubs['observer'] = self.create_publisher(
            tmr4243_interfaces.msg.Observer, '/CSEI/observer/state', 1
        )

        self.dead_reckon_srv = self.create_service(
            std_srvs.srv.Empty, '/CSEI/observer/dead_reckoning', self.dead_reckoning_callback
        )

        self.dead_reckon = False

        self.last_transform = None
        self.last_joystick_msg = None
        self.last_eta_msg = None
        self.last_odom_msg = None
        self.last_tau_msg = None

        self.observer = Observer(L1, L2, L3)

        self.observer_runner = self.create_timer(0.1, self.observer_loop)

    def observer_loop(self):
        #self.get_logger().info("Observer Loop")
        if \
                self.last_eta_msg is None or \
                self.last_tau_msg is None:
            return
        
        if self.dead_reckon:
            eta = self.observer.dead_reckoning()
            eta_msg = std_msgs.msg.Float32MultiArray()
            eta_msg.data = eta
            eta_hat, nu_hat, bias_hat = self.observer.step(
                eta_msg,
                self.last_tau_msg.data
            )

        else:   
            #self.get_logger().info("Estimating")
            eta_hat, nu_hat, bias_hat = self.observer.step(
                self.last_eta_msg,
                self.last_tau_msg.data
            )
        #psi_msg = std_msgs.msg.Float32()
        #psi_msg.data = self.luenberg.get_psi()
        #self.pubs['psi'].publish(psi_msg)
        
        obs = tmr4243_interfaces.msg.Observer()
        obs.eta = eta_hat
        obs.nu = nu_hat
        obs.bias = bias_hat
        self.pubs['observer'].publish(obs)
        #self.get_logger().info(f"Bias: {obs.bias}")

    def tau_callback(self, msg: std_msgs.msg.Float32MultiArray):
        self.last_tau_msg = msg
        #self.get_logger().info(f"Tau: {msg}")

    def eta_callback(self, msg: std_msgs.msg.Float32MultiArray):
        self.last_eta_msg = msg
        #self.get_logger().info(f"Eta: {msg}")

    def odom_callback(self, msg: Odometry):
        self.last_odom_msg = msg

    def dead_reckoning_callback(self, request, response):
        # log the messga in ros2 log console
        self.dead_reckon = not self.dead_reckon
        self.get_logger().info(f"Dead Reckoning Service Called {self.dead_reckon}")

        return response
        


def main():
    rclpy.init()

    node = ObserverNode()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
