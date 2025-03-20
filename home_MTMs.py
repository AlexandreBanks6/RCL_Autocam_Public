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
    
    mtmr = dvrk.mtm("MTMR")
    #Get mtmr pose
    mtmr_pose = pm.toMatrix(mtmr.measured_cp())
    # Both psm1 and psm3 pose wrt to ECM
    psm1_pose = pm.toMatrix(psm1.measured_cp())
    psm3_pose = pm.toMatrix(psm3.measured_cp())

    # Find psm1 pose wrt to psm3
    psm1_wrt_psm3 = inv(psm3_pose) * psm1_pose
    # Set MTMR desired orientation wtr surgeon console to psm1 wtr psm3 orientation
    mtmr_pose[0:3, 0:3] = psm1_wrt_psm3[0:3, 0:3]

    #Convert desired MTMR pose back to pyKDL frame
    mtmr_frame = pm.fromMatrix(mtmr_pose)
    mtmr.move_cp(mtmr_frame).wait()
