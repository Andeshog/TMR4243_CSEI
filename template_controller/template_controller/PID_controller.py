#!/usr/bin/env python3

import numpy as np

class PID:
    def __init__(self):
        self.error_sum = np.zeros(3)

        self.tau_max = np.array([100, 100, 100])

        self.Kp = np.diag([23.04, 34.56, 4.032])
        self.Ki = np.diag([2.76, 4.1, 0.48])
        self.Kd = np.diag([28.7648, 43.1472, 5.048384])

    def step(self, eta_d, eta, eta_dot, dt):
        error = eta - eta_d
        self.error_sum += error * dt
        self.error_sum = np.clip(self.error_sum, -20, 20)

        p = self.Kp @ error
        i = 0#self.Ki @ self.error_sum
        d = self.Kd @ eta_dot

        self.last_error = error

        tau = -(p + i + d)

        if tau[0] > self.tau_max[0]:
            tau[0] = self.tau_max[0]
        elif tau[0] < -self.tau_max[0]:
            tau[0] = -self.tau_max[0]
        
        if tau[1] > self.tau_max[1]:
            tau[1] = self.tau_max[1]
        elif tau[1] < -self.tau_max[1]:
            tau[1] = -self.tau_max[1]

        if tau[2] > self.tau_max[2]:
            tau[2] = self.tau_max[2]
        elif tau[2] < -self.tau_max[2]:
            tau[2] = -self.tau_max[2]

        return tau

