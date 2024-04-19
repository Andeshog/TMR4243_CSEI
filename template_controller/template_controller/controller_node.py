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
import tmr4243_interfaces.msg
import geometry_msgs.msg
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from template_controller.PID_controller import PID
from template_controller.PD_FF_controller import PD_FF_controller
from template_controller.backstepping_controller import BacksteppingController

class Controller(rclpy.node.Node):
    def __init__(self):
        super().__init__("cse_controller")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pubs = {}
        self.subs = {}

        self.subs["reference"] = self.create_subscription(
            tmr4243_interfaces.msg.Reference, '/CSEI/control/reference', self.received_reference, 10)
        
        self.subs["DP_reference"] = self.create_subscription(
            tmr4243_interfaces.msg.Reference, '/CSEI/control/DPref', self.received_dp_ref, 10)

        self.subs['observer'] = self.create_subscription(
            tmr4243_interfaces.msg.Observer, '/CSEI/observer/state', self.received_observer ,10)

        self.pubs["tau_cmd"] = self.create_publisher(
             geometry_msgs.msg.Wrench, '/CSEI/control/tau_cmd', 1)

        # Default starting value is set to 0
        self.P_gain = self.declare_parameter("P_gain",0)
        self.I_gain = self.declare_parameter("I_gain",0)
        self.D_gain = self.declare_parameter("D_gain",0)
        self.K1_gain = self.declare_parameter("K1_gain", 1)
        self.K2_gain = self.declare_parameter("K2_gain", 1)

        self.backstep = BacksteppingController()
        self.pid = PID()

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False

        #self.current_controller  = self.declare_parameter('current_controller', 'PID_controller')
        self.current_controller  = self.declare_parameter('current_controller', 'backstepping_controller')
        self.current_controller.value

        self.last_observer = None
        self.last_transform = None
        self.reference = None
        self.DPref = None
        self.eta_hat_dot = np.zeros(3)
        # timer_period = 0.1 # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        controller_period = 0.1 # seconds
        self.controller_timer = self.create_timer(controller_period, self.controller_callback)


    # def timer_callback(self):

    #     try:
    #         self.last_transform = self.tf_buffer.lookup_transform(
    #             "base_link",
    #             "world",
    #             rclpy.time.Time())
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform : {ex}')

    #     self.current_controller = self.get_parameter('current_controller')


    #     self.get_logger().info(f"Parameter task: {self.current_controller.value}", throttle_duration_sec=1.0)


    def controller_callback(self):

        if self.last_observer is not None and self.DPref is not None:

            eta_hat = np.array(self.last_observer.eta)

            nu_hat = np.array(self.last_observer.nu)

            eta_d = np.array(self.DPref.eta_d)

            tau = self.pid.step(eta_d, eta_hat, nu_hat, 0.1)

            f = geometry_msgs.msg.Wrench()
            f.force.x = tau[0]
            f.force.y = tau[1]
            f.torque.z = tau[2]
            self.pubs["tau_cmd"].publish(f)

            self.last_observer = None

        elif self.last_observer is not None and self.reference is not None:
            #self.get_logger().info("Going into controller callback")

            P_gain = self.get_parameter("P_gain").value
            I_gain = self.get_parameter("I_gain").value
            D_gain = self.get_parameter("D_gain").value
            K1_gain = self.get_parameter("K1_gain").value
            K2_gain = self.get_parameter("K2_gain").value

            tau = self.backstep.control_law(self.last_observer, self.reference)

            # if "PD_FF_controller" in self.current_controller.value:
            #     tau = PD_FF_controller(self.last_observer, self.reference, P_gain, D_gain)

            # elif "PID_controller" in self.current_controller.value:
            #     tau = PID_controller(self.last_observer, self.reference, P_gain, I_gain, D_gain)

            # elif "backstepping_controller" in self.current_controller.value:
            #     tau = self.backstep.control_law(self.last_observer, self.reference)
            #     self.get_logger().info(f"tau: {tau}", throttle_duration_sec=1.0)

            if len(tau) != 3:
                self.get_logger().warn(f"tau has length of {len(tau)} but it should be 3", throttle_duration_sec=1.0)

            f = geometry_msgs.msg.Wrench()
            f.force.x = tau[0]
            f.force.y = tau[1]
            f.torque.z = tau[2]
            self.pubs["tau_cmd"].publish(f)

            self.last_observer = None

        else:

            f = geometry_msgs.msg.Wrench()
            self.pubs["tau_cmd"].publish(f)



    def received_reference(self, msg):
        self.reference = msg

    def received_dp_ref(self, msg):
        self.DPref = msg

    def received_observer(self, msg):
        self.last_observer = msg

    @staticmethod
    def rotationMatrix(psi):
        R = np.array([[np.cos(psi), -np.sin(psi), 0],
                [np.sin(psi), np.cos(psi), 0],
                [0, 0, 1]])
        return R


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

