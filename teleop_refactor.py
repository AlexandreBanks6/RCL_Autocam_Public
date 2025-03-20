import numpy as np
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R
from sensors import footpedal
import sys
import argparse
from numpy import linalg as LA
import filteringUtils

## Important notes:
## For MTM motion:
    # Position: 3D position of MTML (w.r.t to surgeon console) is used to compute the relative translation
    #           applied to PSM1 position (w.r.t to **PSM3**)
    # Orientation: 3D orientation of PSM1 w.r.t PSM3 should always match the orientation of MTML w.r.t 
    #              surgeon console. When user is not driving the PSM (user not present or pressing clutch)
    #              MTML orientation (w.r.t surgeon console) moves to match PSM1 orientation (w.r.t PSM3).
    #              When user is driving PSM1 using MTML (i.e follow mode), PSM3 orientation (w.r.t PSM3)
    #              moves to match MTML orientation (w.r.t surgeon console).
    # Gripper: 1-1 Relationship



# Helper Functions for teleop class

def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

def pickup_transform(offset):

    psm3_T_cam = PyKDL.Frame().Identity()
    psm3_T_cam.p[2] += offset

    return psm3_T_cam

def axis_angle_offset(alignment_offset):

    alignment_offset_angle, _ = alignment_offset.GetRotAngle()
    return alignment_offset_angle * (180/np.pi)

# class Teleop:


## States:

## When the state is distabled, then don't run teloperation

class teleop:
    teleopFrameDelay = 5
    teleopFrameCounter = teleopFrameDelay
    # configuration of the arms
    def configure(self, 
                  parent_arm, 
                  child_arm, 
                  controller_arm, 
                  expected_interval,
                  ignore_operator,
                  scale_factor,
                  cam_offset):
        
        print_id('configuring arms for teleoperation')
        
        ## Configuration of arms
        self.parent_arm = dvrk.psm(arm_name = parent_arm,
                                   expected_interval = expected_interval)
        
        self.child_arm = dvrk.psm(arm_name = child_arm,
                                   expected_interval = expected_interval)
        
        self.controller_arm = dvrk.mtm(arm_name = controller_arm,
                                   expected_interval = expected_interval)

        self.sensors = footpedal()
        ## Select controller (For now keep this blank)
        # self.controller = controller

        self.expected_interval = expected_interval
        self.ignore_operator = ignore_operator
        self.cam_offset = cam_offset
        self.scale_factor = scale_factor
        self.start_teleop = False


    def setting_arm_state(self, arm):

        print_id('starting enable')
        if not arm.enable(10):
            print_id('failed to enable %s' % arm.name())
            sys.exit('failed to enable within 10 seconds')

        print_id('starting home')
        if not arm.home(10):
            print_id('failed to home %s' % arm.name())
            sys.exit('failed to home within 10 seconds')

        print_id('%s home and enable complete' % arm.name())
        
    def set_all_arms_state(self):

        self.setting_arm_state(self.parent_arm)
        self.setting_arm_state(self.child_arm)
        self.setting_arm_state(self.controller_arm)

    def run_get(self):
        
        print_id('Read all the relevant transformations.')
        ## Get all relevant transforms to use

        self.parent_T_cam = pickup_transform(self.cam_offset)
        self.ecm_T_child = self.child_arm.setpoint_cp()
        self.ecm_T_parent = self.parent_arm.setpoint_cp()
        self.hrsv_T_controller = self.controller_arm.measured_cp()

    
    def align_Controller_Arm(self):
        
        print_id('Aligning controller arm to match pose of child arm with respect to parent arm.')

        ecm_T_cam = self.ecm_T_parent * self.parent_T_cam
        cam_T_ecm = ecm_T_cam.Inverse()

        cam_T_child = cam_T_ecm * self.ecm_T_child
        self.hrsv_T_controller.M = cam_T_child.M

        self.controller_arm.move_cp(self.hrsv_T_controller).wait()

        rospy.sleep(2)

        align_offset_initial = self.controller_arm.measured_cp().M.Inverse() * cam_T_child.M

        self.align_offset = align_offset_initial

        alignment_offset_angle_initial = axis_angle_offset(align_offset_initial)

        print_id('Initial Offset Angle: %s Degrees' % str(round(alignment_offset_angle_initial, 3)))

        return alignment_offset_angle_initial


    def check_teleop_start(self):

        if self.ignore_operator == True:

            while self.start_teleop == False:
                try:
                    gripper_angle=self.controller_arm.gripper.measured_js()[0]
                except:
                    print("Unable to grab MTMR gripper")
                    continue
                #print( gripper_angle * (180/np.pi))
                if gripper_angle * (180/np.pi) < 30.0:
                    self.start_teleop = True

                rospy.sleep(self.expected_interval)

    
        else:

            while self.start_teleop == False:

                if (self.controller_arm.gripper.measured_js()[0] < 30.0) and (self.sensors.get_operator_event()):
                    self.start_teleop = True
                
                rospy.sleep(self.expected_interval)


    def release_controller(self):

        zero_wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.controller_arm.use_gravity_compensation(True)
        self.controller_arm.body.servo_cf(zero_wrench)


    def follow_mode(self):
        
        hrsv_T_controller_ini = self.controller_arm.measured_cp()
        hrsv_T_controller_origin = self.controller_arm.measured_cp()

        ecm_T_child_ini = self.child_arm.setpoint_cp() ## w.r.t ECM
        ecm_T_cam_ini = self.parent_arm.setpoint_cp()*self.parent_T_cam

        clutch_pressed_prev = False

        ###########Init Filters
        ecm_T_child_PositionFilter=filteringUtils.CircularBuffer(size=25,num_elements=3)
        ecm_T_child_OrientationFilter=filteringUtils.CircularBuffer(size=25,num_elements=4)

        ecm_T_child_stationary = ecm_T_child_ini

        #---------------Filter Teleop Frame Position--------------
        pos=ecm_T_child_stationary.p
        ecm_T_child_PositionFilter.append(np.array([pos[0], pos[1], pos[2]]))
        pos_mean = ecm_T_child_PositionFilter.get_mean()
        ecm_T_child_stationary.p=PyKDL.Vector(pos_mean[0],pos_mean[1],pos_mean[2])



        #---------------Filter Teleop Frame Orientation--------------
        ecm_T_child_stationary_R=ecm_T_child_stationary.M
        ecm_T_child_stationary_quaternion=ecm_T_child_stationary_R.GetQuaternion()
        quat = np.array([ecm_T_child_stationary_quaternion[0],ecm_T_child_stationary_quaternion[1], ecm_T_child_stationary_quaternion[2], ecm_T_child_stationary_quaternion[3]])
        quat = ecm_T_child_OrientationFilter.negateQuaternion(quat)
        ecm_T_child_OrientationFilter.append(quat)
        rotationMean = ecm_T_child_OrientationFilter.get_mean()
        rotationMean = ecm_T_child_OrientationFilter.negateQuaternion(rotationMean)
        rotationMean = PyKDL.Rotation.Quaternion(rotationMean[0],rotationMean[1],rotationMean[2],rotationMean[3])
        ecm_T_child_stationary.M=rotationMean

        
        
        while self.start_teleop == True:

            # Check if clutch is pressed or not

            #check rotation between current and origin
            try:
                hrsv_T_controller_curr = self.controller_arm.setpoint_cp()
            except: 
                continue

            rotationFrame = hrsv_T_controller_origin.Inverse()*hrsv_T_controller_curr

            rotationEul = rotationFrame.M.GetEulerZYX()
            print(rotationEul)

            # if not any( x > 0.087 for x in rotationEul):
            #     print("smaller")
            #     hrsv_T_controller_curr.M=hrsv_T_controller_origin.M

            ## DISTANCE BASED METHOD
            try:
                ecm_T_parent_curr = self.parent_arm.measured_cp() ## w.r.t ECM
            except: 
                continue
            ecm_T_cam_curr = ecm_T_parent_curr * self.parent_T_cam
            distance = LA.norm(pm.toMatrix(ecm_T_cam_curr)[0:3, 3] - pm.toMatrix(ecm_T_child_stationary)[0:3, 3])
            if distance > 0.01:
                ecm_T_child_stationary = ecm_T_cam_curr

                #---------------Filter Teleop Frame Position--------------
                pos=ecm_T_child_stationary.p
                ecm_T_child_PositionFilter.append(np.array([pos[0], pos[1], pos[2]]))
                pos_mean = ecm_T_child_PositionFilter.get_mean()
                ecm_T_child_stationary.p=PyKDL.Vector(pos_mean[0],pos_mean[1],pos_mean[2])



                #---------------Filter Teleop Frame Orientation--------------
                ecm_T_child_stationary_R=ecm_T_child_stationary.M
                ecm_T_child_stationary_quaternion=ecm_T_child_stationary_R.GetQuaternion()
                quat = np.array([ecm_T_child_stationary_quaternion[0],ecm_T_child_stationary_quaternion[1], ecm_T_child_stationary_quaternion[2], ecm_T_child_stationary_quaternion[3]])
                quat = ecm_T_child_OrientationFilter.negateQuaternion(quat)
                ecm_T_child_OrientationFilter.append(quat)
                rotationMean = ecm_T_child_OrientationFilter.get_mean()
                rotationMean = ecm_T_child_OrientationFilter.negateQuaternion(rotationMean)
                rotationMean = PyKDL.Rotation.Quaternion(rotationMean[0],rotationMean[1],rotationMean[2],rotationMean[3])
                ecm_T_child_stationary.M=rotationMean
            


                
            ## TIME BASED METHOD
            # if self.teleopFrameCounter >= self.teleopFrameDelay:
            #     ecm_T_parent_curr = self.parent_arm.setpoint_cp() ## w.r.t ECM
            #     ecm_T_cam_curr = ecm_T_parent_curr * self.parent_T_cam
            #     self.teleopFrameCounter = 0
            
            # else:
            #     self.teleopFrameCounter+=1 

            ecm_T_child_next = ecm_T_child_ini

            # if self.sensors.flip == False:
            # ecm_T_child_next.M = ecm_T_cam_curr.M * hrsv_T_controller_curr.M * self.align_offset

            #ecm_T_child_next.M = ecm_T_cam_curr.M * hrsv_T_controller_curr.M * self.align_offset * (ecm_T_cam_curr.M.Inverse() * ecm_T_cam_ini.M).Inverse() # Initial from Sayem
            
            
            #check if n amount of iterations have occured since last set 
            ecm_T_child_next.M = ecm_T_child_stationary.M *hrsv_T_controller_curr.M * self.align_offset

            # else:
            #     ecm_T_child_next.M = hrsv_T_controller_curr.M * self.align_offset
            
            controller_translation = self.scale_factor * (hrsv_T_controller_curr.p - hrsv_T_controller_ini.p)

            # ecm_T_child_next.p = ecm_T_child_next.p + ecm_T_cam_curr.M * controller_translation

            print(clutch_pressed_prev)
            if self.sensors.clutch_pressed == True:
                
                if clutch_pressed_prev == False:
                    self.controller_arm.lock_orientation_as_is()
                    clutch_pressed_prev = True
                
            else:

                ecm_T_child_next.p = ecm_T_child_next.p + ecm_T_child_stationary.M * controller_translation

                if clutch_pressed_prev == True:
                    self.controller_arm.unlock_orientation()
                    clutch_pressed_prev = False
                    
                self.child_arm.move_cp(ecm_T_child_next)
                self.child_arm.jaw.move_jp(np.array([0.0]))
                ecm_T_child_ini = ecm_T_child_next

            rospy.sleep(self.expected_interval)

            hrsv_T_controller_ini = hrsv_T_controller_curr
            ecm_T_cam_ini = ecm_T_cam_curr

    def run(self):

        while not rospy.is_shutdown():
            
            self.set_all_arms_state()
            self.run_get()
            self.align_Controller_Arm()
            print("Aligned Arms")
            self.check_teleop_start()
            self.release_controller()
            print("*************")
            print("Started Teleoperation")
            self.follow_mode()

            rospy.sleep(self.expected_interval)
        

if __name__ == '__main__':
    
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_teleop', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--parent', type=str, default='PSM3',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'parent arm name corresponding to ROS topics without namespace.')

    parser.add_argument('-c', '--child', type=str, default='PSM1',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'child arm name corresponding to ROS topics without namespace.')

    parser.add_argument('-m', '--controller', type=str, default='MTMR',
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'controller arm name corresponding to ROS topics without namespace.')
    
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')

    parser.add_argument('-o', '--operator', type=bool, default=True,
                        help = 'Whether headsensor would be considered for teleoperation.')

    parser.add_argument('-s', '--scale', type=float, default=0.4,
                        help = 'Scale factor for teleoperation.')
    
    parser.add_argument('-d', '--offset', type=float, default=0.045,
                        help = 'Offset of camera tip from arm tip.')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = teleop()
    application.configure(args.parent, 
                          args.child, 
                          args.controller, 
                          args.interval, 
                          args.operator, 
                          args.scale, 
                          args.offset)
    application.run() 