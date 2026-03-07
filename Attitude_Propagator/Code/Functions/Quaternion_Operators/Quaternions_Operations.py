

import numpy as np
from Functions.Quaternion_Operators.x_ import x_


def q_mult(q1, q2):
    """
    q_mult computes the quaternion resulting from the multiplication q1 x q2.

    INPUTS
    ----------
    q1, q2 : array-like of shape (4,)
        Input quaternions in the form [eps1, eps2, eps3, eta], being eps the vectorial part and eta the scalar part.

    OUTPUTS
    -------
    q_out : ndarray of shape (4,)
        Resulting quaternion [eps1, eps2, eps3, eta].
    """
    q = np.asarray(q1).flatten()
    r = np.asarray(q2).flatten()

    if q.size != 4 or r.size != 4:
        raise ValueError("Both input quaternions must have 4 components.")

    q1_, q2_, q3_, q0 = q
    r1_, r2_, r3_, r0 = r

    n0 = r0*q0 - r1_*q1_ - r2_*q2_ - r3_*q3_
    n1 = r0*q1_ + r1_*q0 - r2_*q3_ + r3_*q2_
    n2 = r0*q2_ + r1_*q3_ + r2_*q0 - r3_*q1_
    n3 = r0*q3_ - r1_*q2_ + r2_*q1_ + r3_*q0

    q_out = np.array([n1, n2, n3, n0], dtype=float)

    return q_out




def q_conj(q):
    """
    q_conj computes the conjugate of a quaternion.

    INPUTS
    ----------
    q : array-like of shape (4,)
        Input quaternion [eps1, eps2, eps3, eta] being eps the vectorial part and eta the scalar part.

    OUTPUTS
    -------
    q_conjugate : ndarray of shape (4,)
        Conjugate of the input quaternion [-eps1, -eps2, -eps3, eta]
    """
    q = np.asarray(q).flatten()
    if q.size != 4:
        raise ValueError("Input quaternion must have 4 components.")

    eps = -q[:3]  
    eta = q[3]    

    q_conjugate = np.array([*eps, eta], dtype=float)
    return q_conjugate





def q_inv(q):
    """
    q_inv computes the inverse of a quaternion.

    INPUTS
    ----------
    q : array-like of shape (4,)
        Input quaternion [eps1, eps2, eps3, eta] being eps the vectorial part and eta the scalar part.

    OUTPUTS
    -------
    q_inverse : ndarray of shape (4,)
        Inverse of the input quaternion.
    """
    q = np.asarray(q).flatten()
    if q.size != 4:
        raise ValueError("Input quaternion must have 4 components.")

    q_conjugate = q_conj(q)
    q_norm_sq = np.sum(q**2)  
    q_inverse = q_conjugate / q_norm_sq

    return q_inverse

