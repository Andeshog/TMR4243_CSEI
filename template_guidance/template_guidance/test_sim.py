from stationkeeping import ReferenceFilter
from template_controller.PID_controller import PID
import numpy as np
import matplotlib.pyplot as plt

pid = PID()
refFilter = ReferenceFilter()

eta_ref = np.array([5, 5, 0])

T = 100
dt = 0.1
time = np.arange(0, T, dt)
x_d = np.zeros((len(time), 9))

noise = np.random.normal(0, 0.1, (len(time), 3))
current = np.ones((len(time), 3))

tau = np.zeros((len(time), 3))
eta = np.zeros((len(time), 3))
eta[0] = np.array([3, 3, 0])
x_d[0,0:3] = eta[0]
x_d[0,3:6] = np.array([0.1, 0.1, 0])
eta_dot = np.zeros(3)
nu = np.zeros((len(time), 3))
nu[0] = np.array([0.25, 0.25, 0])

M = np.array([
            [16, 0, 0],
            [0, 24, 0.53],
            [0, 0.53, 2.8]
            ])
D = np.array([
            [0.66, 0, 0],
            [0, 1.3, 2.8],
            [0, 0, 1.9]
            ])

M_diag = np.diag(np.diag(M))
D_diag = np.diag(np.diag(D))

## PID TUNING VALUES ##
omega_n = 1.2
zeta = 0.75
Kp = M_diag * omega_n**2
Kd = M_diag * 2 * zeta * omega_n #- D_diag
Ki = omega_n/10 * Kp

pid.Kp = Kp
pid.Kd = Kd
pid.Ki = Ki

# Simulation

for i in range(1,len(time)):
    x_d[i] = refFilter.step(eta_ref, x_d[i-1])
    eta_d = x_d[i,0:3]
    tau[i] = pid.step(eta_d, eta[i-1, :], nu[i-1, :], dt)

    # Simulate the system
    nu_dot = np.linalg.inv(M)@(tau[i] - D@nu[i-1])
    eta_dot = ReferenceFilter.rotationMatrix(eta[i-1,2])@nu[i-1]
    eta[i] = eta[i-1] + eta_dot*dt
    nu[i] = nu[i-1] + nu_dot*dt

plt.figure()
plt.plot(time, eta[:,0], label='North')
plt.plot(time, x_d[:,0], label='North ref')
plt.legend()
plt.grid()

plt.figure()
plt.plot(time, eta[:,1], label='East')
plt.plot(time, x_d[:,1], label='East ref')
plt.legend()
plt.grid()

plt.figure()
plt.plot(time, eta[:,2], label='Psi')
plt.plot(time, x_d[:,2], label='Psi ref')
plt.legend()
plt.grid()

plt.figure()
plt.plot(time, tau[:,0], label='tau_1')
plt.plot(time, tau[:,1], label='tau_2')
plt.plot(time, tau[:,2], label='tau_3')
plt.legend()
plt.grid()

plt.show()


