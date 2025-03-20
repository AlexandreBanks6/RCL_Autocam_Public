import numpy as np
import dvrk
import PyKDL
import rospy
import sys
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA
import pickle

if sys.version_info.major < 3:
    input = raw_input

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()

    # arm_pose = arm.setpoint_cp()
    # arm_pose.M = PyKDL.Frame().Identity().M
    # arm.move_jp(arm_pose).wait()\
    arm_joints = arm.measured_js()[0]
    arm_joints[3:6] = [ 0.0, 0.0, 0.0]
    arm.move_jp(arm_joints).wait()
    jawAng = arm.jaw.measured_js()
    jawAng[0] = 0.0
    arm.jaw.move_jp(np.array([0.0]))


def calibrate(psm1, psm3):


    # psm1.move_cp(psm1_Identity_pose).wait()
    # psm3.move_cp(psm3_Identity_pose).wait()
    psm1_pose = []
    psm3_pose = []

    n = 5
    print(f"Now collecting {n} points...")
    for i in range(n):

        print(" Move PSM1 & PSM3 to touch the next fiducial.")
        input("    Press Enter to continue...")
        psm1_pose.append(psm1.measured_cp())
        psm3_pose.append(psm3.measured_cp())

    print("Finished Calibration")



    pickle.dump(psm1_pose, open("psm1_pose.p", "wb")) 
    pickle.dump(psm3_pose, open("psm3_pose.p", "wb")) 


if __name__ == '__main__':

    print("Initializing arms...")

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")

    setting_arms_state(psm1)
    setting_arms_state(psm3)

    calibrate(psm1, psm3)





