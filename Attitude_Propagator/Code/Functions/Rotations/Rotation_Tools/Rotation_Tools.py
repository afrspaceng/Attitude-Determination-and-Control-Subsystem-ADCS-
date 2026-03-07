import numpy as np
from Functions.Quaternion_Operators.Quaternions_Operations import q_mult, q_inv, q_conj  
from Functions.Quaternion_Operators.x_ import x_ 
from Functions.Rotations.Principal_Rotations.Rotations import Cx, Cy, Cz



def q_from_rot(phi, v):
    """
    q_from_rot returnes the quaternion corresponding to a rotation of
    an angle phi (in degrees) about a specified axis v.

    INPUTS
    ----------
    phi : float
        Rotation angle in degrees.

    v : array-like of shape (3,)
        3D rotation axis vector (does not need to be unitary).

    OUTPUT
    ----------
    q : ndarray of shape (4,)
        Quaternion [q1, q2, q3, q4], where:
            - q[0:3] are the vector components (eps)
            - q[3]   is the scalar component (eta)

    NOTE
    ----------
    - The axis vector 'v' is automatically normalized, and so, the quaternion is returned as a unit quaternion.
    
    """
    
    v = np.asarray(v).flatten()
    if v.size != 3:
        raise ValueError("Input axis 'v' must have 3 components.")
    
    # Normalize axis
    v = v / np.linalg.norm(v)
    
    phi_rad = np.deg2rad(phi)
    
    eta = np.cos(phi_rad / 2)
    eps = np.sin(phi_rad / 2) * v
    
    q = np.hstack((eps, eta))
    
    return q




def q_Rot_Comp(q1, q2):
    """
    q_Rot_Comp computes the quaternion that represents the combined rotation
    obtained by first applying q1 and then q2.

    INPUTS
    ----------
    q1, q2 : array-like of shape (4,)
        Quaternions in the form [eps1, eps2, eps3, eta], where:
            - eps = [q1, q2, q3] is the vector part
            - eta   is the scalar part
        Each quaternion must be of unit norm.

        q1 represents rotation from frame A to B
        q2 represents rotation from frame B to C

    OUTPUT
    ----------
    q_out : ndarray of shape (4,)
        Quaternion representing rotation from frame A to C [eps; eta].
    """
    
    q1 = np.asarray(q1).flatten()
    q2 = np.asarray(q2).flatten()
    
    if q1.size != 4 or q2.size != 4:
        raise ValueError("Both input quaternions must have 4 components.")
    
    eps1, eta1 = q1[:3], q1[3]
    eps2, eta2 = q2[:3], q2[3]
    
    eps_out = eta2 * eps1 + eta1 * eps2 + x_(eps1).dot(eps2)
    eta_out = eta2 * eta1 - np.dot(eps2, eps1)
    
    q_out = np.hstack((eps_out, eta_out))
    
    return q_out




def q_to_DCM(q):
    """
    q_to_DCM computes the rotation matrix corresponding to a quaternion.

    INPUTS
    ----------
    q : array-like of shape (4,)
        Input quaternion [eps1, eps2, eps3, eta], being eps the vectorial part and eta the scalar part.

    OUTPUTS
    -------
    C_matrix : ndarray of shape (3,3)
        Rotation matrix associated with the quaternion.
    """
    q = np.asarray(q).flatten()
    if q.size != 4:
        raise ValueError("Input quaternion must have 4 components.")

    eps = q[:3]   
    eta = q[3]    

    C_matrix = (2*eta**2 - 1) * np.eye(3) + 2 * np.outer(eps, eps) - 2 * eta * x_(eps)

    return C_matrix




def v_rot_q(v, q):
    """
    v_rot_q rotates a 3D vector using the quaternion q.

    This function rotates a 3D vector 'v' by the rotation represented by 
    the quaternion 'q'. The resulting vector 'v_rotated' is expressed 
    in the same reference frame as 'v'.

    INPUTS
    ----------
    v : array-like of shape (3,)
        3D vector to be rotated.

    q : array-like of shape (4,)
        Quaternion [q1, q2, q3, q4], where:
            - q[0:3] are the vector components (eps)
            - q[3]   is the scalar component (eta)

    OUTPUT
    ----------
    v_rotated : ndarray of shape (3,)
        Rotated vector, expressed in the same frame as 'v'.

    NOTES
    ----------
    If the quaternion 'q' is not unitary, it will be normalized first.
    The rotation formula used is:
        v_rotated = 2*(eps dot v)*eps + (eta^2 - eps dot eps)*v + 2*eta*(eps cross v)
    """
    
    v = np.asarray(v).flatten()
    q = np.asarray(q).flatten()
    
    if v.size != 3:
        raise ValueError("Input vector 'v' must have 3 components.")
    if q.size != 4:
        raise ValueError("Quaternion 'q' must have 4 components.")
    
    # Normalize quaternion if it is not unitary
    norm_q = np.linalg.norm(q)
    if abs(norm_q - 1) > 1e-6:
        eta = q[3]
        phi = 2 * np.rad2deg(np.arccos(eta))   
        axis = q[:3] / np.sin(np.deg2rad(phi/2))
        q = q_from_rot(phi, axis)  
    
    eps = q[:3]  
    eta = q[3]   
    
    v_rotated = (2 * np.dot(eps, v) * eps
                 + (eta**2 - np.dot(eps, eps)) * v
                 + 2 * eta * np.cross(eps, v))
    
    return v_rotated




def q_to_EulerA(q):
    """
    q_to_EulerA computes Roll (X), Pitch (Y) and Yaw (Z) angles from a quaternion in 3(z)-2(y)-1(x) rotation configuration.

    INPUTS
    ----------
    q : array-like of shape (4,)
        Quaternion [q1, q2, q3, q4], where:
            - q[0:3] are the vector part (eps)
            - q[3]   is the scalar part (eta)

    OUTPUTS
    ----------
    P : float
        Pitch angle (rotation around Y-axis) in degrees.

    Y : float
        Yaw angle (rotation around Z-axis) in degrees.

    R : float
        Roll angle (rotation around X-axis) in degrees.

    NOTES
    ----------
    The function assumes the quaternion is in the form [eps1, eps2, eps3, eta].
    """
    q = np.asarray(q).flatten()
    if q.size != 4:
        raise ValueError("Input quaternion must have 4 components.")
    
    eps = q[:3]
    eta = q[3]
    q1, q2, q3 = eps
    q4 = eta
    
    # Roll 
    R = np.rad2deg(np.arctan2(2*(q4*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
    
    # Pitch 
    P = np.rad2deg(np.arcsin(2*(q4*q2 - q3*q1)))
    
    # Yaw
    Y = np.rad2deg(np.arctan2(2*(q4*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))
    
    return P, Y, R




def DCM_to_EulerA(C):
    """
    DCM_to_EulerA converts a Direction Cosine Matrix (DCM) to Euler angles (Yaw, Pitch, Roll)
    using a 3-2-1 rotation sequence (Z-Y-X).

    Rotation sequence:
        1. Yaw   (psi)   - rotation about the Z-axis
        2. Pitch (theta) - rotation about the Y-axis
        3. Roll  (phi)   - rotation about the X-axis

    INPUT:
        C : array-like of shape (3,3)
            Direction Cosine Matrix (DCM). Must be a valid rotation matrix (orthogonal with det(C)=1).

    OUTPUT:
        theta : float - Pitch angle in degrees
        psi   : float - Yaw angle in degrees
        phi   : float - Roll angle in degrees
    """
    C = np.asarray(C)
    if C.shape != (3, 3):
        raise ValueError("Input C must be a 3x3 matrix.")
    
    theta = -np.degrees(np.arcsin(C[0, 2]))
    psi   = np.degrees(np.arctan2(C[0, 1], C[0, 0]))
    phi   = np.degrees(np.arctan2(C[1, 2], C[2, 2]))
    
    return theta, psi, phi




def DCM_to_q(C):
    """
    DCM_to_q converts a Direction Cosine Matrix (DCM) to a quaternion.

    INPUT:
        C : array-like of shape (3,3)
            Direction Cosine Matrix. Must be a valid rotation matrix.

    OUTPUT:
        q : ndarray of shape (4,)
            Quaternion [eps1, eps2, eps3, eta], where:
                eps = vector part (first three elements)
                eta = scalar part (fourth element)
    """
    C11, C12, C13 = C[0, 0], C[0, 1], C[0, 2]
    C21, C22, C23 = C[1, 0], C[1, 1], C[1, 2]
    C31, C32, C33 = C[2, 0], C[2, 1], C[2, 2]

    eta = 0.5 * np.sqrt(1 + C11 + C22 + C33)

    # REFERENCE: Spacecraft Dynamics and Control. Anton H.J. de Ruiter. Chapter 1.
    if np.isclose(eta, 0):
        eps1 = np.sqrt((C11 + 1) / 2)
        eps2 = np.sqrt((C22 + 1) / 2)
        eps3 = np.sqrt((C33 + 1) / 2)

        if abs(eps1) > 0:
            eps1 = abs(eps1)
            eps2 = np.sign(C12) * eps2
            eps3 = np.sign(C13) * eps3
        elif abs(eps2) > 0:
            eps1 = np.sign(C12) * eps1
            eps2 = abs(eps2)
            eps3 = np.sign(C23) * eps3
        elif abs(eps3) > 0:
            eps1 = np.sign(C13) * eps1
            eps2 = np.sign(C23) * eps2
            eps3 = abs(eps3)
    else:
        eps1 = (C23 - C32) / (4 * eta)
        eps2 = (C31 - C13) / (4 * eta)
        eps3 = (C12 - C21) / (4 * eta)

    q = np.array([eps1, eps2, eps3, eta])

    return q



def EulerA_to_DCM(psi, theta, phi):
    """
    EulerA_to_DCM converts Euler angles (Yaw, Pitch, Roll) into a Direction Cosine Matrix (DCM)
    using a 3-2-1 rotation sequence (Z-Y-X).

    INPUTS:
        psi   : float - Yaw angle in degrees
        theta : float - Pitch angle in degrees
        phi   : float - Roll angle in degrees

    OUTPUT:
        C : ndarray of shape (3,3)
            Direction Cosine Matrix representing the rotation
    """
    C = Cx(phi) @ Cy(theta) @ Cz(psi)
    return C



def EulerA_to_q(psi, theta, phi):
    """
    EulerA_to_q converts Euler angles (Yaw, Pitch, Roll) in 3-2-1 rotation sequence
    into the corresponding quaternion.

    Rotation sequence: Z-Y-X
        psi   : Yaw   - rotation about Z-axis in degrees
        theta : Pitch - rotation about Y-axis in degrees
        phi   : Roll  - rotation about X-axis in degrees

    OUTPUT:
        q : ndarray of shape (4,)
            Quaternion [eps1, eps2, eps3, eta], with eps as vector part and eta as scalar part
    """
    cpsi = np.cos(np.deg2rad(psi / 2))
    cth  = np.cos(np.deg2rad(theta / 2))
    cphi = np.cos(np.deg2rad(phi / 2))
    
    spsi = np.sin(np.deg2rad(psi / 2))
    sth  = np.sin(np.deg2rad(theta / 2))
    sphi = np.sin(np.deg2rad(phi / 2))
    
    eta   = cpsi * cth * cphi + spsi * sth * sphi
    eps1  = sphi * cth * cpsi - cphi * sth * spsi
    eps2  = cphi * sth * cpsi + sphi * cth * spsi
    eps3  = cphi * cth * spsi - sphi * sth * cpsi
    
    q = np.array([eps1, eps2, eps3, eta], dtype=float)
    return q
