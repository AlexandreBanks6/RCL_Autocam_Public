#!/usr/bin/env python

import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
import rospy
from numpy.linalg import inv
import footpedals

if __name__ == '__main__':
    
    # Teleoperation scale factor
    scale_factor = 0.2

    rospy.init_node('dvrk_origin_test', anonymous=True)
    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    console = dvrk.console('dVRK')
    mtmr = dvrk.mtm("MTMR")
    footpedal = footpedals.footpedal()

    #Get start mtmr pose, we overwrite this every iteration, setting it to mtmr_pos(n-1)
    mtmr_prev_pose = pm.toMatrix(mtmr.measured_cp())
    # print(footpedal.get_headsensor_state())
    while not rospy.is_shutdown():

        operator_present = footpedal.get_headsensor_state()

        while(operator_present == True):
            # Both psm1 and psm3 pose wrt to ECM
            psm1_pose = pm.toMatrix(psm1.measured_cp())
            psm3_pose = pm.toMatrix(psm3.measured_cp())

            # Get current MTMR pose
            mtmr_curr_pose = pm.toMatrix(mtmr.measured_cp())
            mtmr_displacement = mtmr_curr_pose[0:3, -1] - mtmr_prev_pose[0:3, -1]
            mtmr_orientation = mtmr_curr_pose[0:3, 0:3]

            
            # Find psm1 pose wrt to psm3
            psm1_wrt_psm3 = inv(psm3_pose) * psm1_pose
            # Set MTMR desired orientation wtr surgeon console to psm1 wtr psm3 orientation
            psm1_wrt_psm3[0:3, 0:3] = mtmr_orientation[0:3, 0:3]
            psm1_wrt_psm3[0:3, -1] = psm1_wrt_psm3[0:3, -1] + scale_factor * mtmr_displacement

            # Compute the resulting desired psm1 pose wrt to ECM
            psm1_wrt_ecm = psm3_pose * psm1_wrt_psm3

            #Convert desired psm1 pose wrt ECM back to pyKDL frame
            psm1_frame = pm.fromMatrix(psm1_wrt_ecm)
            psm1.move_cp(psm1_frame).wait()

            #Set previous mtmr position to current position
            mtmr_prev_pose = mtmr_curr_pose

        
    psm1.move_cp(psm1_frame).wait()
