import numpy as np
from Functions.Quaternion_Operators.x_ import x_

def Attitude_Kinematics(t, y, sim_input):
    """
    Attitude_Kinematics computes the derivative of the state vector
    for the satellite's attitude dynamics.

    INPUTS
    ----------
    t : float
        Current time [s]. 
    
    y : array-like of shape (7,)
        State vector containing:
            - Quaternion q = [q1, q2, q3, q4] (q4 = scalar part)
            - Angular velocity omega = [omegax, omegay, omegaz] [rad/s]
    
    sim_input : object
        Simulation input with the following attributes:
            - Inertia_Matrix : 3x3 inertia matrix of the satellite
            - perturbations : 3x1 vector of external torques [Nm]

    OUTPUT
    ----------
    dydt : ndarray of shape (7,)
        Derivative of the state vector: [dot_eps; dot_eta; dot_omega]
    """
    
    # Satellite and environment properties
    I = sim_input.Inertia_Matrix
    pert = np.asarray(sim_input.perturbations).flatten()
    
    # Quaternion. Normalizing if necessary
    q = np.asarray(y[:4]).flatten()
    q_norm_error = np.linalg.norm(q) - 1
    if abs(q_norm_error) > 1e-6:
        q = q / np.linalg.norm(q)
    
    eps = q[:3]  
    eta = q[3]   
    
    # Angular velocity
    omega = np.asarray(y[4:7]).flatten()
    
    # Quaternion rate
    dot_eps = 0.5 * (eta * np.eye(3) + x_(eps)) @ omega
    dot_eta = -0.5 * np.dot(eps, omega)
    
    # Angular velocity rate
    dot_omega = np.linalg.solve(I, pert - x_(omega) @ (I @ omega)).flatten()
    
    # State derivative vector
    dydt = np.concatenate([dot_eps, [dot_eta], dot_omega])
    
    return dydt





