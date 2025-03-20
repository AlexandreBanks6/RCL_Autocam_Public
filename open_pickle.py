import pickle
import dvrk
import PyKDL
import tf_conversions.posemath as pm
import numpy as np

psm1_pose = pickle.load(open("psm1_pose.p", "rb"))
psm3_pose = pickle.load(open("psm3_pose.p", "rb"))

psm1_coords = []
psm3_coords = []

from math import sqrt

def tip_transform(tip_offset):

    psm_T_tip = PyKDL.Frame().Identity()
    psm_T_tip.p[2] += tip_offset

    return psm_T_tip

scaling = False

# Implements Kabsch algorithm - best fit.
# Supports scaling (umeyama)
# Compares well to SA results for the same data.
# Input:
#     Nominal  A Nx3 matrix of points
#     Measured B Nx3 matrix of points
# Returns s,R,t
# s = scale B to A
# R = 3x3 rotation matrix (B to A)
# t = 3x1 translation vector (B to A)
def rigid_transform_3D(A, B, scale):
    assert len(A) == len(B)

    N = A.shape[0];  # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # center the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    if scale:
        H = np.transpose(BB) * AA / N
    else:
        H = np.transpose(BB) * AA

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T

    if scale:
        varA = np.var(A, axis=0).sum()
        c = 1 / (1 / varA * np.sum(S))  # scale factor
        t = -R * (centroid_B.T * c) + centroid_A.T
    else:
        c = 1
        t = -R * centroid_B.T + centroid_A.T

    return c, R, t

if __name__ == '__main__':

    print("Initializing arms...")
    err = []
    print("PSM1 Pose")

    psm1_tip_offset = 2.1/100
    psm3_tip_offset = 2.8/100

    psm1_T_tip = tip_transform(psm1_tip_offset)
    psm3_T_tip = tip_transform(psm3_tip_offset)

    psm1_pose = pickle.load(open("psm1_pose.p", "rb"))
    psm3_pose = pickle.load(open("psm3_pose.p", "rb"))

    for p in range(len(psm1_pose)):

        ecm_T_psm1_tip = psm1_pose[p] * psm1_T_tip
        ecm_T_psm3_tip = ecm_T_psm1_tip
        ecm_T_psm3_tip.M = psm3_pose[p].M

        ecm_T_psm3_theoretical = ecm_T_psm3_tip * psm3_T_tip.Inverse() 

        offset = pm.toMatrix(ecm_T_psm3_theoretical)[0:3, 3] - pm.toMatrix(psm3_pose[p])[0:3, 3]
        err.append(offset)

    offset = np.sum(np.array(err), axis=0) / len(psm1_pose)
    offset_vec = PyKDL.Vector(offset[0], offset[1],  offset[2])
    print("Calibration Offset")
    print(offset)

    psm1 = dvrk.psm("PSM1")
    psm1.enable()
    psm1.home()
    psm3 = dvrk.psm("PSM3")
    psm3.enable()
    psm3.home()

    for i in range(4):

        input("    Press Enter after moving PSM1...")
        psm1_pose = psm1.setpoint_cp()
        
        input("    Press Enter to move PSM3 to this point...")
        psm3_compute_pose = psm3.setpoint_cp()

        psm3_compute_pose.p = psm1_pose.p - offset_vec
        # psm3_pose.M = psm1_pose.M
        psm3.move_cp(psm3_compute_pose).wait()

# print(psm1_positions.shape)
