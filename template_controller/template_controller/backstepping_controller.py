#!/usr/bin/env python3

import numpy as np
from tmr4243_interfaces.msg import Reference, Observer

class BacksteppingController:
    def __init__(self, K1: np.ndarray, K2: np.ndarray) -> None:
        self.K_1 = K1
        self.K_2 = K2
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

    def control_law(self, observer: Observer, reference: Reference) -> np.ndarray:
        """
        Calculates the control input based on the observer estimate and the reference.

        Args:
            observer (Observer): The estimated state of the system.
            reference (Reference): The reference to follow.

        Returns:
            np.ndarray: The control input.
        """

        # Extract values from the state and reference
        eta = observer[:3]
        nu = observer[3:6]
        bias = observer[6:9]
        w = reference.w
        v_s = reference.v_s
        v_ss = reference.v_ss
        eta_d = np.array([reference.eta_d[0], reference.eta_d[1], reference.eta_d[2]])
        eta_d_s = np.array([reference.eta_ds[0], reference.eta_ds[1], reference.eta_ds[2]])
        eta_d_ss = np.array([reference.eta_ds2[0], reference.eta_ds2[1], reference.eta_ds2[2]])

        # Get R_transposed and S
        R_trps = self.rotationmatrix_in_yaw_transpose(eta[2])
        S = self.skew_symmetric_matrix(nu[2])

        # Define error signals
        eta_error = eta - eta_d
        eta_error[2] = self.ssa(eta_error[2])

        z1 = R_trps @ eta_error
        alpha1 = -self.K_1 @ z1 + R_trps @ eta_d_s * v_s

        z2 = nu - alpha1

        sigma1 = self.K_1 @ (S @ z1) - self.K_1 @ nu - S @ (R_trps @ eta_d_s) * v_s

        ds_alpha1 = self.K_1 @ (R_trps @ eta_d_s) + R_trps @ eta_d_ss * v_s + R_trps @ eta_d_s * v_ss

        # Control law ## Må endres om de ulineære matrisene skal brukes
        tau = -self.K_2 @ z2 + self.D @ nu + self.M @ sigma1 + self.M @ ds_alpha1 * (v_s + w) - bias

        # Add constraints to tau # This should be improved
        # for i in range(len(tau)):
        #     if tau[i] > self.tau_max[i]:
        #         tau[i] = self.tau_max[i]
        #     elif tau[i] < -self.tau_max[i]:
        #         tau[i] = -self.tau_max[i]

        return tau

    @staticmethod
    def rotationmatrix_in_yaw_transpose(psi: float) -> np.ndarray:
        R = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
        R_trps = np.transpose(R)
        return R_trps
    
    @staticmethod
    def skew_symmetric_matrix(r: float) -> np.ndarray:
        S = np.array([[0, -r, 0],
                    [r, 0, 0],
                    [0, 0, 0]])
        return S
    
    @staticmethod
    def ssa(angle: float) -> float:
        angle = np.atan2(np.sin(angle), np.cos(angle))
        return angle
