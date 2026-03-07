#--------------------------------------------------------------------------
# Rotation matrices around X, Y, Z axes
#--------------------------------------------------------------------------

import numpy as np

def Cx(theta):
    """
    Cx computes the rotation matrix for a rotation around the X-axis.

    This function returns a 3x3 rotation matrix that rotates a vector 
    counterclockwise around the X-axis by an angle `theta` (in degrees).

    INPUTS
    ----------
    theta : float
        Rotation angle in degrees.

    OUTPUTS
    -------
    Rotation : ndarray of shape (3,3)
        Rotation matrix following the standard right-hand rule.
    """
    rad = np.deg2rad(theta)  
    Rotation = np.array([
        [1,          0,           0],
        [0,  np.cos(rad),  np.sin(rad)],
        [0, -np.sin(rad),  np.cos(rad)]
    ], dtype=float)
    return Rotation


def Cy(theta):
    """
    Cy computes the rotation matrix for a rotation around the Y-axis.

    This function returns a 3x3 rotation matrix that rotates a vector 
    counterclockwise around the Y-axis by an angle `theta` (in degrees).

    INPUTS
    ----------
    theta : float
        Rotation angle in degrees.

    OUTPUTS
    -------
    Rotation : ndarray of shape (3,3)
        Rotation matrix following the standard right-hand rule.
    """
    rad = np.deg2rad(theta)
    Rotation = np.array([
        [ np.cos(rad), 0, -np.sin(rad)],
        [          0, 1,           0],
        [ np.sin(rad), 0,  np.cos(rad)]
    ], dtype=float)
    return Rotation


def Cz(theta):
    """
    Cz computes the rotation matrix for a rotation around the Z-axis.

    This function returns a 3x3 rotation matrix that rotates a vector 
    counterclockwise around the Z-axis by an angle `theta` (in degrees).

    INPUTS
    ----------
    theta : float
        Rotation angle in degrees.

    OUTPUTS
    -------
    Rotation : ndarray of shape (3,3)
        Rotation matrix following the standard right-hand rule.
    """
    rad = np.deg2rad(theta)
    Rotation = np.array([
        [ np.cos(rad),  np.sin(rad), 0],
        [-np.sin(rad),  np.cos(rad), 0],
        [           0,           0, 1]
    ], dtype=float)
    return Rotation





