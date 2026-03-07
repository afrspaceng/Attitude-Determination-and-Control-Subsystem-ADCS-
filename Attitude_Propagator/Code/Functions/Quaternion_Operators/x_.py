import numpy as np

def x_(vector):
    """
    x_ computes the skew-symmetric matrix of a 3-component vector.

    The skew-symmetric matrix [v]_x of a vector v = [v1, v2, v3]^T is defined as:
        [  0  -v3   v2 ]
        [ v3    0  -v1 ]
        [-v2   v1    0 ]

    INPUTS
    ----------
    vector : array-like of shape (3,)
        Input 3-component vector (list, tuple, or ndarray).

    OUTPUTS
    -------
    skew_matrix : ndarray of shape (3,3)
        The skew-symmetric matrix corresponding to the input vector.
        
    """
    vector = np.asarray(vector).flatten()
    if vector.size != 3:
        raise ValueError("Input vector must have 3 components.")

    skew_matrix = np.array([
        [0,        -vector[2],  vector[1]],
        [vector[2], 0,         -vector[0]],
        [-vector[1], vector[0], 0       ]
    ], dtype=float)

    return skew_matrix





