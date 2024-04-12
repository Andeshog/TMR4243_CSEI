 #!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
import numpy as np
from template_observer.wrap import wrap
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

class Observer:
    def __init__(self, L1, L2, L3):

        self.M = np.array([
                    [16, 0, 0],
                    [0, 24, 0.53],
                    [0, 0.53, 2.8]
                    ])
        self.D = np.array([
                    [0.66, 0, 0],
                    [0, 1.3, 2.8],
                    [0, 0, 1.9]
                    ])
        
        self.L1 = 5*np.diag(L1)
        self.L2 = 0.01*np.diag(L2) #@ self.M
        self.L3 = 0.001*np.diag(L3) #@ self.M

        self.eta_hat =   np.array([0, 0, 0])
        self.nu_hat  =   np.array([0, 0, 0])
        self.bias_hat   =   np.array([0, 0, 0])

        self.delta_t = 0.1

    def step(self, eta: Float32MultiArray, tau):#odom: Odometry, tau):

        # eta = self.odom_to_state(odom)

        eta = np.array(eta.data)

        Tb = np.diag([142, 142, 142])
        self.psi = eta[2]
        psi_wrapped = wrap(self.psi)
        y_tilde = eta - self.eta_hat
        R_trsp = self.rotation_matrix_inverse(psi_wrapped)

        eta_hat_dot = R_trsp @ self.nu_hat + self.L1 @ y_tilde
        nu_hat_dot = np.linalg.inv(self.M) @ (-self.D @ self.nu_hat + self.bias_hat + tau + self.L2 @ R_trsp @ y_tilde)
        bias_hat_dot = -np.linalg.inv(Tb) @ self.bias_hat + self.L3 @ R_trsp @ y_tilde

        # eta_hat_dot = np.matmul(R_trsp, self.nu_hat) + np.matmul(self.L1, y_tilde)
        # nu_hat_dot = np.matmul(np.linalg.inv(self.M), (-np.matmul(self.D, self.nu_hat) + self.bias_hat + tau + np.matmul(self.L2, np.matmul(R_trsp, y_tilde))))
        # bias_hat_dot = -np.matmul(np.linalg.inv(Tb), self.bias_hat) + np.matmul(self.L3, np.matmul(R_trsp, y_tilde))

        self.eta_hat = self.eta_hat + eta_hat_dot * self.delta_t
        self.nu_hat = self.nu_hat + nu_hat_dot * self.delta_t
        self.bias_hat = self.bias_hat + bias_hat_dot * self.delta_t

        return self.eta_hat.tolist(), self.nu_hat.tolist(), self.bias_hat.tolist()
    
    def dead_reckoning(self):
        eta = np.zeros(3)

        surge_vel = self.nu_hat[0]
        sway_vel = self.nu_hat[1]
        yaw_rate = self.nu_hat[2]

        psi_hat = self.eta_hat[2]

        # delta_x = surge_vel * np.sin(psi_hat) * self.delta_t - sway_vel * np.cos(psi_hat) * self.delta_t
        # delta_y = surge_vel * np.cos(psi_hat) * self.delta_t + sway_vel * np.sin(psi_hat) * self.delta_t
        # delta_psi = yaw_rate * self.delta_t

        delta_x = surge_vel * np.cos(psi_hat) * self.delta_t - sway_vel * np.sin(psi_hat + np.pi/2) * self.delta_t
        delta_y = surge_vel * np.sin(psi_hat) * self.delta_t + sway_vel * np.cos(psi_hat + np.pi/2) * self.delta_t
        delta_psi = yaw_rate * self.delta_t

        eta[0] = self.eta_hat[0] + delta_x
        eta[1] = self.eta_hat[1] + delta_y
        eta[2] = self.eta_hat[2] + delta_psi

        # eta[2] = wrap(eta[2])

        return eta.tolist()
    
    def get_psi(self):
        return self.psi

    @staticmethod
    def rotation_matrix_inverse(psi):
        R =  np.array([[np.cos(psi), -np.sin(psi), 0],
                     [np.sin(psi), np.cos(psi), 0],
                     [0, 0, 1]])
        R_transpose = np.transpose(R)
        return R_transpose
    
    @staticmethod
    def odom_to_state(msg: Odometry) -> np.ndarray:
        """
        Converts an Odometry message to a state 3DOF vector.

        Args:
            msg (Odometry): The Odometry message to convert.

        Returns:
            np.ndarray: The state vector.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]

        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = quat2euler(orientation_list)

        state = np.array([x, y, yaw])

        return state
