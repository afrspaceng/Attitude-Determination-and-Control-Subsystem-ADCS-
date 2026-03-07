# %% TEST 1
import numpy as np
from Functions.Rotations.Principal_Rotations.Rotations import Cx, Cy, Cz  # Adjust path if necessary
from Functions.Quaternion_Operators.x_ import x_ 
from Functions.Quaternion_Operators.Quaternions_Operations import q_conj, q_mult, q_inv
from Functions.Rotations.Rotation_Tools.Rotation_Tools import q_from_rot, q_Rot_Comp, q_to_DCM, v_rot_q, q_to_EulerA

# Vector in the initial frame (1)
v = np.array([1, 1, 0])

# First check
C21 = Cz(45)
C32 = Cy(34)
C31 = np.dot(C32, C21)
Vector_en_2 = np.dot(C31, v)

print("Vector in frame 2:", Vector_en_2)
# Validated

############################################################################################################################################
# Second check
axis1 = np.array([0, 0, 1])
angle1 = 45
q21 = q_from_rot(angle1, axis1)

axis2 = np.array([0, 1, 0])
angle2 = 34
q32 = q_from_rot(angle2, axis2)

q31 = q_Rot_Comp(q21, q32)

C_Q = q_to_DCM(q31)

print("Second check:")
print("Combined quaternion q31:", q31)
print("Rotation matrix derived from the quaternion:\n", C_Q)
print()
print("Note: the quaternion represents the same information as the C31 matrix.")
print("      For rotations, the rotation axis must be unitary from the start.")
# Validated

##############################################################################################################################################
# %% Third check
v_new = v_rot_q(v, q21)

q_v = q_mult(q21, np.array([v[0], v[1], v[2], 0]))
v_new_2 = q_mult(q_v, q_conj(q21))


# Validated. The result is exactly the same.
print("v_new  =", v_new)
print("v_new_2 =", v_new_2[:3])


# %% Fifth check
Pitch, Yaw, Roll = q_to_EulerA(q31)

print("Pitch =", Pitch)
print("Yaw   =", Yaw)
print("Roll  =", Roll)

# Validated
