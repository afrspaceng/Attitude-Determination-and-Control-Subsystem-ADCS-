from math import pi
import numpy as np
import matplotlib.pyplot as plt
from Functions.Solver.Propagation import Propagation

# -----------------------------
# 1. Simulation parameters
# -----------------------------
time = 1000
dt = 1

#The user can use any function from "Functions.Rotations.Rotation_Tools" to give initial conditions as a DCM, quaternion or Euler Angles.
q0 = np.array([0.020859650045003,0.750436163417316,-0.326605551162254,-0.574229396186315])
omega0 = np.array([1*2*pi/360, 2*2*pi/360, 0])

I = np.array([[1, 0.1, 0.1],
              [0.1, 2, 0.1],
              [0.1, 0.1, 0.3]])

perturbations = [0, 0, 0]
sim_input = type('', (), {})()  
sim_input.Inertia_Matrix = I
sim_input.perturbations = perturbations

# -----------------------------
# 2. Propagation
# -----------------------------
sol = Propagation(q0, omega0, time, dt, sim_input)

# -----------------------------
# 3. Plots
# -----------------------------

# Define colors
gris_1 = [0, 0.12, 0.25]
gris_2 = [0.87, 0.72, 0.53]
gris_3 = [0, 0.5, 0]

# --------------------------
# Figure 1: Angular velocity components 
# --------------------------
plt.figure(1)
plt.plot(sol['t'], sol['omegax'] * 360 / (2 * np.pi), color=gris_1, linestyle='-', linewidth=1.1, label=r'$\omega_x$')
plt.plot(sol['t'], sol['omegay'] * 360 / (2 * np.pi), color=gris_2, linestyle='-', linewidth=1.1, label=r'$\omega_y$')
plt.plot(sol['t'], sol['omegaz'] * 360 / (2 * np.pi), color=gris_3, linestyle='--', linewidth=1.1, label=r'$\omega_z$')

plt.xlabel(r'$t$ [s]', fontsize=13)
plt.ylabel(r'$\omega_{bG}^b$ [deg/s]', fontsize=13)
plt.title('Angular Velocity Components', fontsize=15)
plt.grid(True)
plt.legend(fontsize=12)
plt.xticks(fontsize=13)
plt.yticks(fontsize=13)

# --------------------------
# Figure 2: Euler angles
# --------------------------
plt.figure(2)
plt.plot(sol['t'], sol['Yaw'], color=gris_1, linestyle='-', linewidth=1.1, label=r'$\psi$')
plt.plot(sol['t'], sol['Pitch'], color=gris_2, linestyle='-', linewidth=1.1, label=r'$\theta$')
plt.plot(sol['t'], sol['Roll'], color=gris_3, linestyle='--', linewidth=1.1, label=r'$\phi$')

plt.xlabel(r'$t$ [s]', fontsize=13)
plt.ylabel('Euler angle [deg]', fontsize=13)
plt.title('Euler Angles', fontsize=15)
plt.grid(True)
plt.legend(fontsize=12)
plt.xticks(fontsize=13)
plt.yticks(fontsize=13)

# --------------------------
# Figure 3: Quaternion components
# --------------------------
plt.figure(3)
plt.plot(sol['t'], sol['eps1'], color=gris_1, linestyle='-', linewidth=1.1, label=r'$\epsilon_1$')
plt.plot(sol['t'], sol['eps2'], color=gris_2, linestyle='-', linewidth=1.1, label=r'$\epsilon_2$')
plt.plot(sol['t'], sol['eps3'], color=gris_3, linestyle='--', linewidth=1.1, label=r'$\epsilon_3$')
plt.plot(sol['t'], sol['eta'],  color='#80B3FF', linestyle='--', linewidth=1.1, label=r'$\eta$')

plt.xlabel(r'$t$ [s]', fontsize=13)
plt.ylabel('Quaternion component [-]', fontsize=13)
plt.title('Quaternion', fontsize=15)
plt.grid(True)
plt.legend(fontsize=12)
plt.xticks(fontsize=13)
plt.yticks(fontsize=13)

plt.show()
