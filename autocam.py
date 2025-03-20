import numpy as np
import dvrk
import PyKDL
import rospy
import sys
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from numpy import linalg as LA
import pickle
from std_msgs.msg import Bool

if sys.version_info.major < 3:
    input = raw_input
    
def pickup_transform(cam_offset):

    psm3_T_cam = PyKDL.Frame().Identity()
    psm3_T_cam.p[2] += cam_offset

    return psm3_T_cam

def flip(is_flipped):
    pub = rospy.Publisher('isflipped', Bool, queue_size=1)
    pub.publish(is_flipped)

def ring_transform(ring_offset):

    psm1_T_R = PyKDL.Frame().Identity()
    psm1_T_R.p[2] += ring_offset

    return psm1_T_R

def tip_transform(tip_offset):

    psm_T_tip = PyKDL.Frame().Identity()
    psm_T_tip.p[2] += tip_offset

    return psm_T_tip

def load_and_set_calibration():

    psm1_calib_pose = pickle.load(open("psm1_pose.p", "rb"))
    psm3_calib_pose = pickle.load(open("psm3_pose.p", "rb"))

    err = []

    psm1_tip_offset = 2.1/100
    psm3_tip_offset = 2.8/100

    psm1_T_tip = tip_transform(psm1_tip_offset)
    psm3_T_tip = tip_transform(psm3_tip_offset)

    for p in range(len(psm1_calib_pose)):

        ecm_T_psm1_tip = psm1_calib_pose[p] * psm1_T_tip
        ecm_T_psm3_tip = ecm_T_psm1_tip
        ecm_T_psm3_tip.M = psm3_calib_pose[p].M

        ecm_T_psm3_theoretical = ecm_T_psm3_tip * psm3_T_tip.Inverse() 

        offset = pm.toMatrix(ecm_T_psm3_theoretical)[0:3, 3] - pm.toMatrix(psm3_calib_pose[p])[0:3, 3]
        err.append(offset)

    offset = np.sum(np.array(err), axis=0) / len(psm1_calib_pose)
    offset_vec = PyKDL.Vector(offset[0], offset[1],  offset[2])

    return offset_vec


def orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset):

    y_R = pm.toMatrix(ecm_T_R)[0:3, 1]

    y_R = y_R / LA.norm(y_R)
    z_i = z_i / LA.norm(z_i)
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    
    # sgn1 =  np.sign(np.dot(z_i, y_R)) 
    sgn2 = np.sign(np.dot(z_w, y_R))

    # print(sgn1 * sgn2)

    z_cam = - sgn2 * y_R 


    x_cam = np.cross(z_w, z_cam) / LA.norm(np.cross(z_w, z_cam))
    y_cam = np.cross(z_cam, x_cam) / LA.norm(np.cross(z_cam, x_cam))

    x_cam_vec = PyKDL.Vector(x_cam[0], x_cam[1], x_cam[2])
    y_cam_vec = PyKDL.Vector(y_cam[0], y_cam[1], y_cam[2])
    z_cam_vec = PyKDL.Vector(z_cam[0], z_cam[1], z_cam[2])

    ecm_R_cam = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)


    ecm_p_cam = ecm_T_R.p - df * z_cam_vec

    ecm_T_cam_desired = PyKDL.Frame(ecm_R_cam, ecm_p_cam)
    ecm_T_psm3_desired = ecm_T_cam_desired * psm3_T_cam.Inverse()
    ecm_T_psm3_desired.p = ecm_T_psm3_desired.p  - offset

    return ecm_T_psm3_desired, sgn2


def compute_intermediate(psm3_T_cam, ecm_T_R, ecm_T_w, df, offset):

    x_R = pm.toMatrix(ecm_T_R)[0:3, 0]

    x_R = x_R / LA.norm(x_R)
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    sgn = np.sign(np.dot(z_w, x_R))

    z_cam =  - sgn * x_R 


    x_cam = np.cross(z_w, z_cam) / LA.norm(np.cross(z_w, z_cam))
    y_cam = np.cross(z_cam, x_cam) / LA.norm(np.cross(z_cam, x_cam))

    x_cam_vec = PyKDL.Vector(x_cam[0], x_cam[1], x_cam[2])
    y_cam_vec = PyKDL.Vector(y_cam[0], y_cam[1], y_cam[2])
    z_cam_vec = PyKDL.Vector(z_cam[0], z_cam[1], z_cam[2])

    ecm_R_cam = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)


    ecm_p_cam = ecm_T_R.p - 1.2 * df * z_cam_vec

    ecm_T_cam_desired = PyKDL.Frame(ecm_R_cam, ecm_p_cam)
    ecm_T_psm3_desired = ecm_T_cam_desired * psm3_T_cam.Inverse()
    ecm_T_psm3_desired.p = ecm_T_psm3_desired.p  - offset

    return ecm_T_psm3_desired

## When the state is distabled, then don't run teloperation
def disabled():
    return None

## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()
        
# def initial_align(psm3, start_pose):
    
if __name__ == '__main__':

    print("Initializing arms...")
    
    rospy.init_node('dvrk_teleop', anonymous=True)

    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    ecm = dvrk.ecm("ECM")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    setting_arms_state(ecm)

    ecm_pose = ecm.setpoint_cp()

    ring_offset = 0.033 ## 1.5 cm
    cam_offset = 0.045 ## 4 cm
    df = 0.12 ## in cms
    ## HARD CODED OFFSET FOR GIVEN JOINT CONFIGURATION
    
    offset = load_and_set_calibration()

    # Find respective transforms from psm1 to ring and from psm3 to cam
    psm1_T_R = ring_transform(ring_offset)
    psm3_T_cam = pickup_transform(cam_offset)    
    ecm_T_w = ecm.setpoint_cp().Inverse()

    message_rate = 0.01

    ## For first iteration, we need to gracefully park PSM3 at the start of our tracking...
    # Query the poses for psm1, psm3 and ecm
    psm1_pose = psm1.setpoint_cp()
    psm3_pose = psm3.setpoint_cp()

    ecm_T_R = psm1_pose * psm1_T_R

    z_i = pm.toMatrix(psm3_pose)[0:3, 2]

    ecm_T_psm3_desired_Pose, prev_sgn = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)
    
    print("Parking PSM3 to starting position...")
    fixed_posed = ecm_T_psm3_desired_Pose.p
    psm3.move_cp(ecm_T_psm3_desired_Pose).wait()

    input("    Press Enter to start autonomous tracking...")
    
    ## For every iteration:
    while not rospy.is_shutdown():

        # Query the poses for psm1, psm3 and ecm
        psm1_pose = psm1.setpoint_cp()
        psm3_pose = psm3.setpoint_cp()

        ecm_T_R = psm1_pose * psm1_T_R

        z_i = pm.toMatrix(psm3_pose)[0:3, 2]

        ecm_T_psm3_desired_Pose, curr_sgn = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)

        if prev_sgn != curr_sgn:
            flip(False)
            ecm_T_psm3_intermediate = compute_intermediate(psm3_T_cam, ecm_T_R, ecm_T_w, df, offset)
            print("Flipping")
            psm3.move_cp(ecm_T_psm3_intermediate).wait()
            psm3.move_cp(ecm_T_psm3_desired_Pose).wait()
            # rospy.sleep(message_rate)

        else:
            flip(True)
            psm3.move_cp(ecm_T_psm3_desired_Pose)
            rospy.sleep(message_rate)


        prev_sgn = curr_sgn



## Initial Starting Position of PSM1: -4.634, -2.615, 240.000, 59.881, 13.880, -66.092   Jaw: 10.000