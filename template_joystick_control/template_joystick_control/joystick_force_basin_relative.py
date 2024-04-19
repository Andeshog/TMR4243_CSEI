#!/usr/bin/env python3

import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np

def joystick_force_basin_relative(joystick: sensor_msgs.msg, position: geometry_msgs.msg.TransformStamped):
    # Replace the following line
    u0, u1, u2, a1, a2 = 0, 0, 0, 0, 0

    # Transformation from body forces to basin
    psi_mc = position.rotation[2]
    rot = np.array([[np.cos(psi_mc), -np.sin(psi_mc), 0],
                    [np.sin(psi_mc), np.cos(psi_mc), 0],
                    [0, 0, 1]]) # from body to NED

    # Extract body forces
    tau_x = joystick.axes[1]
    tau_y = joystick.axes[0]
    tau_yaw = joystick.axes[3]

    # Body forces
    tau = np.array([tau_x, tau_y, tau_yaw])

    # NED forces
    tau_ned = rot @ tau

    # Thrust Allocation Matrix
    B = np.array([[0, 1, 0, 1, 0],
                  [1, 0, 1, 0, 1],
                  [0.3875, 0.055, -0.4754, -0.055, -0.4574]])

    # Simple Moore-Penrose Pseudoinverse
    B_ps = np.linalg.pinv(B)

    # Compute total thrust outputs
    f = B_ps @ tau_ned

    # Compute body forces and directions
    u1 = np.sqrt(f[1]**2 + f[2]**2)
    u2 = np.sqrt(f[3]**2 + f[4]**2)
    u0 = f[0]

    a1 = np.arctan2(f[2], f[1])
    a2 = np.arctan2(f[4], f[3])

    return (u0, u1, u2, a1, a2)