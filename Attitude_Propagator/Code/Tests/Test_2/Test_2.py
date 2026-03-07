
import numpy as np
from Functions.Rotations.Principal_Rotations.Rotations import Cx, Cy, Cz
from Functions.Rotations.Rotation_Tools.Rotation_Tools import DCM_to_q, DCM_to_EulerA, EulerA_to_DCM, EulerA_to_q, q_to_DCM, q_to_EulerA, q_Rot_Comp, v_rot_q, q_from_rot

# --------------------------
# Test 2
# --------------------------

# 1. Create a rotation matrix C21.
C21 = Cx(122) @ Cy(246) @ Cz(190)

# 2. Compute the quaternions for each rotation.
q1 = q_from_rot(190, [0, 0, 1])
q2 = q_from_rot(246, [0, 1, 0])
q3 = q_from_rot(122, [1, 0, 0])

# Compose the quaternions sequentially.
q21_ = q_Rot_Comp(q1, q2)
q21 = q_Rot_Comp(q21_, q3)

# 3. Compute the rotation matrix from the composed quaternion.
C = q_to_DCM(q21)

# 4. Compute the quaternion from the rotation matrix.
q_new = DCM_to_q(C)

# 5. Extract Pitch, Yaw, Roll (PYR) from the quaternion.
P1, Y1, R1 = q_to_EulerA(q21)

# 6. Extracting Pitch, Yaw, Roll (PYR) from the rotation matrix.
P2, Y2, R2 = DCM_to_EulerA(C21)

# 7. They are the same.
print("PYR from quaternion: Pitch={}, Yaw={}, Roll={}".format(P1, Y1, R1))
print("PYR from DCM:        Pitch={}, Yaw={}, Roll={}".format(P2, Y2, R2))
# Validated

# 8. Sensor
Cbs: np.ndarray = Cy(-90) @ Cx(180)

qsi: np.ndarray = np.array([0.936680, 0.216195, 0.124597, 0.245695])

Csi: np.ndarray = q_to_DCM(qsi)

Cbi: np.ndarray = Cbs @ Csi  

qbi: np.ndarray = DCM_to_q(Cbi)

P1, Y1, R1 = q_to_EulerA(qbi)

P2, Y2, R2 = DCM_to_EulerA(Cbi)

print("\n===== SENSOR QUESTION RESULTS =====")
print("Body-to-Sensor DCM (Cbs):")
print(Cbs)

print("\nSensor-to-Inertial DCM (Csi):")
print(Csi)

print("\nBody-to-Inertial DCM (Cbi):")
print(Cbi)

print("\nQuaternion qbi (Body-to-Inertial):")
print(qbi)

print("\nEuler Angles from Quaternion [deg] (Yaw, Pitch, Roll):")
print(f"Yaw: {Y1:.3f}, Pitch: {P1:.3f}, Roll: {R1:.3f}")

print("\nEuler Angles from DCM [deg] (Yaw, Pitch, Roll):")
print(f"Yaw: {Y2:.3f}, Pitch: {P2:.3f}, Roll: {R2:.3f}")
print("========================================")

# Validated
