#!/usr/bin/env python3

import numpy as np
import geometry_msgs.msg
import std_msgs.msg

import typing


def thruster_allocation(tau: np.ndarray) -> list[float]:
   B = np.array([[0, 1, 0, 1, 0],
                  [1, 0, 1, 0, 1],
                  [0.3875, 0.055, -0.4574, -0.055, -0.4574]])
    
   W = np.diag([1, 1, 1, 1, 1])
   W_inv = np.linalg.inv(W)
   Ke = np.diag([2.629, 2.629, 1.030, 1.030, 1.030])
   Ke_inv = np.linalg.inv(Ke)

   # Compute pseudo inverse of B
   B_ps = W_inv @ B.T @ np.linalg.inv(B @ W_inv @ B.T)
   #B_ps = np.pinv(B)

   f = B_ps @ tau
   fd = np.array([1, 1, 1, 1, 1])

   Q_W = np.eye(5) - B_ps @ B

   # Compute the control input
   f_star = B_ps @ tau + Q_W @ fd

   #u_e = Ke_inv @ B_ps @ tau

   u1 = np.sqrt(f_star[1]**2 + f_star[2]**2)
   u2 = np.sqrt(f_star[3]**2 + f_star[4]**2)
   u0 = f_star[0]

   a1 = np.arctan2(f_star[2], f_star[1])
   a2 = np.arctan2(f_star[4], f_star[3])
   
   return [u0, u1, u2, a1, a2]

def thrust_allocation_two_thrusters(tau: np.ndarray) -> list[float]:
   B = np.array([[1, 0, 1, 0],
                  [0, 1, 0, 1],
                  [0.055, -0.4574, -0.055, -0.4574]])
    
   W = np.diag([1, 1, 1, 1])
   W_inv = np.linalg.inv(W)
   # Ke = np.diag([2.629, 2.629, 1.030, 1.030, 1.030])
   # Ke_inv = np.linalg.inv(Ke)

   # Compute pseudo inverse of B
   B_ps = W_inv @ B.T @ np.linalg.inv(B @ W_inv @ B.T)
   #B_ps = np.pinv(B)

   f = B_ps @ tau
   fd = np.array([1, 1, 1, 1])

   Q_W = np.eye(4) - B_ps @ B

   # Compute the control input
   f_star = f + Q_W @ fd

   #u_e = Ke_inv @ B_ps @ tau

   u0 = 0.0
   u1 = np.sqrt(f_star[0]**2 + f_star[1]**2)
   u2 = np.sqrt(f_star[2]**2 + f_star[3]**2)

   a1 = np.arctan2(f_star[1], f_star[0])
   a2 = np.arctan2(f_star[3], f_star[2])

   return [u0, u1, u2, a1, a2]
   