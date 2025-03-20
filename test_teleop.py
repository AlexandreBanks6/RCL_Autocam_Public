#!/usr/bin/env python

import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
from numpy.linalg import inv

if __name__ == '__main__':
    rospy.init_node('dvrk_origin_test', anonymous=True)
    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    
    ECM = dvrk.ecm("ECM")
    #Get mtmr pose
    ECM_pose = pm.toMatrix(ECM.measured_cp())
    # Both psm1 and psm3 pose wrt to ECM
    psm1_pose = pm.toMatrix(psm1.measured_cp())
    psm3_pose = pm.toMatrix(psm3.measured_cp())

    # Set ECM pose orientation to same as that of psm3
    ECM_pose[0:3, 0:3] = psm3_pose[0:3, 0:3]

    #Convert desired ECM pose back to pyKDL frame
    ECM_frame = pm.fromMatrix(ECM_pose)

    ECM.move_cp(ECM_frame).wait()
