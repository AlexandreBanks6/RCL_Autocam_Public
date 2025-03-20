import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from sensors import footpedal
import sys
from motion import CmnUtil


TELEOPERATION_SCALE=0.4
START_ANGLE=20.0 #We start teleoperation when user grips both smaller than this


class teleoperation:
    def __init__(self):
        
        #Setting teleop parameters
        # self.ignore_operator=True #True if we ignore headset
        self.teleop_scale=TELEOPERATION_SCALE
        self.start_teleop=False
        self.interval=0.01 #Run interval for rospy

        #Initializes the PSM's and MTMs
        self.psm1=dvrk.psm("PSM1") #Mapped to left hand
        # self.psm3=dvrk.psm("PSM3") #Mapped to right hand
        
        # self.mtmL=dvrk.mtm("MTML")
        self.mtmR=dvrk.mtm("MTMR")

        #Gets the footpedal sensor
        self.sensors=footpedal()

        #Setting the arm states
        CmnUtil.setting_arm_state(self.psm1)
        # utils.setting_arm_state(self.psm3)
        # utils.setting_arm_state(self.mtmL)
        CmnUtil.setting_arm_state(self.mtmR)
        print("************************")
        print("Initialized Teleoperation")

        #Alignment offsets
    
    def run(self):
        #Runs the teleoperation
        while not rospy.is_shutdown():
            self.align_MTMs()   #Starts teleop by aligning
            self.check_teleop_start() #Starts teleop when user grips
            self.release_controller()
            self.follow_mode()
            rospy.sleep(self.interval)

    def align_MTMs(self):
        # self.MTML_offset_rotation,MTML_offset_angle=self.align_MTM(self.mtmL,self.psm3)
        self.MTMR_offset_rotation,MTMR_offset_angle=self.align_MTM(self.mtmR,self.psm1)

        # print("MTML Initial Offset: "+str(MTML_offset_angle)+" (deg)")
        print("MTMR Initial Offset: "+str(MTMR_offset_angle)+" (deg)")
        print("Completed Arm Allignment")
        print("************************")


    def align_MTM(self,MTM,PSM):
        hrsv_T_mtm=MTM.measured_cp()    #Gets current MTM pose
        ecm_T_psm=PSM.setpoint_cp()

        hrsv_T_mtm.M=ecm_T_psm.M    #Enforce that MTM matches PSM orientation

        MTM.move_cp(hrsv_T_mtm).wait() #Moves the orientation to match

        rospy.sleep(2) #Sleeps

        align_offset_transform=MTM.measured_cp().M.Inverse()*ecm_T_psm.M
        align_offset_angle_initial=self.axis_angle_offset(align_offset_transform)

        return align_offset_transform,align_offset_angle_initial
    

    def check_teleop_start(self):
        #Checks to see if pinch has been done
        # mtml_gripped=False
        mtmr_gripped=False
        while self.start_teleop == False:            

            if self.mtmR.gripper.measured_js()[0] * (180/np.pi) < 30.0:
                mtmr_gripped=True
            # if self.mtmL.gripper.measured_js()[0] * (180/np.pi) < 30.0:
            #     mtml_gripped=True

            if mtmr_gripped: #and mtml_gripped:
                self.start_teleop=True
                print("Started Teloperation")
                return

            rospy.sleep(self.interval)   

    def release_controller(self):

        zero_wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.mtmR.use_gravity_compensation(True)
        self.mtmR.body.servo_cf(zero_wrench)

        # self.mtmL.use_gravity_compensation(True)
        # self.mtmL.body.servo_cf(zero_wrench)

        return
    
    def follow_mode(self):
        #Gets initial teleoperation vals
        # hrsv_T_mtml_ini=self.mtmL.measured_cp()
        # ecm_T_psm3_ini=self.psm3.measured_cp()
        try:
            hrsv_T_mtmr_ini=self.mtmR.measured_cp()
        except:
            print("Unable to Read MTMR")
            return
        
        try:
            ecm_T_psm1_ini=self.psm1.measured_cp()
        except:
            print("Unable to Read PSM1")
            return

        clutch_pressed_prev=False

        while self.start_teleop==True:
            #Runs continously
            # hrsv_T_mtml_curr=self.mtmL.setpoint_cp()
            try:
                hrsv_T_mtmr_curr=self.mtmR.setpoint_cp()
            except:
                print("Unable to Read MTMR")
                continue

            # ecm_T_psm3_curr=self.psm3.measured_cp()
            # ecm_T_psm1_curr=self.psm1.measured_cp()


            #Inits objects 
            # ecm_T_psm3_next=ecm_T_psm3_ini
            

            #Sets the ecm_T_psm orientation
            # ecm_T_psm3_next.M=hrsv_T_mtml_curr.M*self.MTML_offset_rotation
            

            #Gets the translation terms
            # mtml_trans=hrsv_T_mtml_curr.p-hrsv_T_mtml_ini.p
            # ecm_T_psm3_next.p=self.teleop_scale*mtml_trans+ecm_T_psm3_ini

            
            

            #Checks the clutch
            if self.sensors.clutch_pressed == True:                
                if clutch_pressed_prev == False:
                    self.mtmR.lock_orientation_as_is()
                    # self.mtmL.lock_orientation_as_is()
                    clutch_pressed_prev = True
            else:
                if clutch_pressed_prev == True:
                    self.mtmR.unlock_orientation()
                    # self.mtmL.unlock_orientation()
                    clutch_pressed_prev = False

                ecm_T_psm1_next=ecm_T_psm1_ini
                ecm_T_psm1_next.M=hrsv_T_mtmr_curr.M*self.MTMR_offset_rotation
                mtmr_trans=hrsv_T_mtmr_curr.p-hrsv_T_mtmr_ini.p
                ecm_T_psm1_next.p=self.teleop_scale*mtmr_trans+ecm_T_psm1_ini.p

                #Moves the arms
                # self.psm3.move_cp(ecm_T_psm3_next)
                # mtml_gripper=self.mtmL.gripper.measured_js()[0] #Gets the joint angle
                # self.mtmL.jaw.move_jp(np.array(mtml_gripper))

                self.psm1.move_cp(ecm_T_psm1_next)
                #mtmr_gripper=self.mtmR.gripper.measured_js()[0] #Gets the joint angle
                self.psm1.jaw.move_jp(np.array([0.0]))
            
            hrsv_T_mtmr_ini=hrsv_T_mtmr_curr
            
            rospy.sleep(self.interval)



    ############Helper Methods######
    def axis_angle_offset(self,alignment_offset):
        alignment_offset_angle, _ = alignment_offset.GetRotAngle()
        return alignment_offset_angle * (180/np.pi)


if __name__=='__main__':
    rospy.init_node('dvrk_teleop',anonymous=True)
    teleop_app=teleoperation()
    teleop_app.run()



        





