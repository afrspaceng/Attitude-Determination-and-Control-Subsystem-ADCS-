import numpy as np
from Functions.Attitude_Kinematics.Attitude_Kinematics import Attitude_Kinematics
from Functions.Numerical_Schemes.RK4 import RK4
from Functions.Rotations.Rotation_Tools.Rotation_Tools import q_to_EulerA

def Propagation(q0, omega0, time, dt, sim_input):
    """
    Propagation - Solves the satellite attitude propagation using RK4.

    This function integrates the satellite's attitude kinematics over time
    using the 4th-order Runge-Kutta (RK4) method. The state vector includes 
    both the quaternion and the angular velocity.

    INPUTS
    ----------
    q0       : array-like of shape (4,)
               Initial quaternion [q1, q2, q3, q4] with scalar part last.
    omega0   : array-like of shape (3,)
               Initial angular velocity [omegax, omegay, omegaz].
    time     : float
               Final time for propagation.
    dt       : float
               Time step size.
    sim_input: object
               Object containing simulation parameters (e.g., inertia matrix, perturbations).

    OUTPUT
    ----------
    sol : dict
          Dictionary containing propagated states and derived quantities:
          - sol['q']      : ndarray (N x 4), quaternion at each time step
          - sol['omega']  : ndarray (N x 3), angular velocity at each time step
          - sol['eps1'], ['eps2'], ['eps3'], ['eta'] : ndarray (N,), quaternion components
          - sol['omegax'], ['omegay'], ['omegaz']   : ndarray (N,), angular velocity components
          - sol['Pitch'], ['Yaw'], ['Roll']         : ndarray (N,), Euler angles from quaternions
          - sol['t']      : ndarray (N,), time vector
    """

    # Ensure column vectors
    q0 = np.asarray(q0).flatten()
    omega0 = np.asarray(omega0).flatten()

    # Initial state vector
    y0 = np.concatenate([q0, omega0])

    # Run RK4 integration
    t, state = RK4(Attitude_Kinematics, y0, time, dt, sim_input)

    # Extract quaternion and angular velocity
    sol = {}
    sol['q'] = state[:, 0:4]
    sol['omega'] = state[:, 4:7]

    sol['eps1'] = state[:, 0]
    sol['eps2'] = state[:, 1]
    sol['eps3'] = state[:, 2]
    sol['eta']  = state[:, 3]

    sol['omegax'] = state[:, 4]
    sol['omegay'] = state[:, 5]
    sol['omegaz'] = state[:, 6]

    # Compute Euler angles at each time step
    N = state.shape[0]
    Pitch = np.zeros(N)
    Yaw   = np.zeros(N)
    Roll  = np.zeros(N)

    for i in range(N):
        q = state[i, 0:4]
        Pitch[i], Yaw[i], Roll[i] = q_to_EulerA(q)

    sol['Pitch'] = Pitch
    sol['Yaw']   = Yaw
    sol['Roll']  = Roll

    # Time vector
    sol['t'] = t

    return sol





