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

psm1_calib_pose = pickle.load(open("psm1_pose.p", "rb"))
psm3_calib_pose = pickle.load(open("psm3_pose.p", "rb"))

err = []

for p in range(len(psm1_calib_pose)):
    err.append(pm.toMatrix(psm1_calib_pose[p])[0:3, 3] - pm.toMatrix(psm3_calib_pose[p])[0:3, 3])

errs = np.sum(np.array(err), axis=0) / len(psm1_calib_pose)

calib_transforms = np.eye(4)
Rotation = np.array([[0.998332076373577,    -0.00421748694954359,  -0.0575784515895905],
                     [0.00180943054157016,   0.999123923386903,    -0.0418104254591807],
                     [0.0577043433784631,    0.0416365046538842,    0.997465092239056]])

err_vec = np.array([-0.0699, -0.0703, 0.26001])

offset = PyKDL.Vector(errs[0], errs[1],   errs[2])

calib_transforms[0:3, 0:3] = Rotation
calib_transforms[0:3, 3] = err_vec
calib_frame = pm.fromMatrix(calib_transforms)
print(calib_frame)

psm1 = dvrk.psm("PSM1")
psm1.enable()
psm1.home()
psm3 = dvrk.psm("PSM3")
psm3.enable()
psm3.home()

for i in range(4):

    input("    Press Enter after moving PSM1...")
    psm1_pose = psm1.setpoint_cp()
    print(psm1_pose)
    
    input("    Press Enter to move PSM3 to this point...")
    psm3_pose = psm3.setpoint_cp()

    psm3_pose.p = psm1_pose.p - offset
    # psm3_pose.M = psm1_pose.M
    psm3.move_cp(psm3_pose).wait()
    

print("Finished Task...")


