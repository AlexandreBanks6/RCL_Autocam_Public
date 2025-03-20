# import dvrkKinematics
import numpy as np
import PyKDL
import rospy
from motion import arm
import tf_conversions.posemath as pm
from geometry_msgs.msg import PoseStamped
import csv
import os
import crtk
import cisstRobotPython
import copy
from motion import CmnUtil
### NOTE: The closed form IK appears to require the remote centre of motion for its computation. This means one must subscribe to /PSM1/local/measured_cp to obtain the PSM1 pose in the frame of the RCM

base_T_PSM_SUJ = []


class PSMmanipulator(cisstRobotPython.robManipulator):
    def __init__(self,*args):
        super().__init__(*args)
        self.ERRORSUCCESS = 0
        self.ERRORFAILURE = 1
        self.SafeDistanceFromRCM = 0.01
        self.tool = ""
        #for PSM
        self.jointsLowerBounds = [-4.7124,-0.9250, 0.00, -4.5379, -1.3963, -1.3963]
        self.jointsUpperBounds = [ 4.7124,0.9250, 0.24,  4.5379,  1.3963,  1.3963]

    def loadTool(self, toolName):
        self.LoadRobot(toolName +".rob")
        self.tool = toolName

    #Assumes that the input joints contains the current joint positions of 
    #the robot
    def InverseKinematics(self, joints, pose, errorThresh, iterations):
        currentJoints = joints
        currentDepth = currentJoints[2]
        joints_copied = copy.deepcopy(joints)

        if self.tool == "PROGRASP_FORCEPS_420093":
            #multiply by the following function because the rotation is is applied to tip of prograsp in JSON
            # but is not present in  PROGRASP_FORCEPS_420093.rob
            # print("check")
            tipRotation = np.array([[ 0.0, -1.0,  0.0,  0.0],
                                    [ 0.0,  0.0,  1.0,  0.0],
                                    [-1.0,  0.0,  0.0,  0.0],
                                    [ 0.0,  0.0,  0.0,  1.0]])
            pose =  pose @  CmnUtil.invHomogeneousNumpy(tipRotation)
        #user the inherited bindings to C++ inverse kinematic solver
        solverError = super().InverseKinematics(joints_copied, pose, errorThresh, iterations)

        #PERFORM MAPPING USED ON THE DVRK API: https://github.com/jhu-cisst/cisst/blob/main/cisstRobot/code/robManipulator.cpp
        if solverError == self.ERRORSUCCESS:
            #find closest solution mod 2 pi
            difference = currentJoints[3] - joints_copied[3]
            differenceInTurns = int(difference/(2.0*np.pi))
            joints_copied[3] = joints_copied[3] + differenceInTurns*2.0*np.pi

            #project away from RCM if not safe, using axis at end of shaft
            f4 = self.ForwardKinematics(joints_copied,4)
            distanceToRCM = np.linalg.norm(f4[0:4, 0:4])
            #if not far enough, distance for axis 4 is fully determine by insertion joint so add to it
            if (distanceToRCM < self.SafeDistanceFromRCM):
                minDepth = joints_copied[2] + (self.SafeDistanceFromRCM - distanceToRCM)
                if (currentDepth <= minDepth):
                    joints_copied[2] = max(currentDepth, joints_copied[2])
                else:
                    #make sure we dont go deeper
                    joints_copied[2] = minDepth

        return joints_copied, solverError
    
        
    def checkJointLimits(self, solverState, q, verbose = False):
        withinLimits = True
        jointViolated = []
        if self.tool == "PROGRASP_FORCEPS_420093":
            if solverState != 0:
                jointViolated.append(-1)
                withinLimits =  False
            if q[0] < -4.7124 or q[0] > 4.7124:
                withinLimits =  False
                jointViolated.append(0)
            if q[1] < -0.9250 or q[1] > 0.9250:
                withinLimits =  False
                jointViolated.append(1)

            if q[2] < 0.00 or q[2] > 0.2400:
                withinLimits =  False
                jointViolated.append(2)

            if q[3] < -4.5379 or q[3] > 4.5379:
                withinLimits =  False
                jointViolated.append(3)

            if q[4] < -1.3963 or q[4] > 1.3963:
                withinLimits =  False
                jointViolated.append(4)
            if q[5] < -1.3963 or q[5] > 1.3963:
                withinLimits =  False
                jointViolated.append(5)
            
            if verbose:
                if withinLimits == False:
                    print("JOINT LIMITS VIOLATED DUE TO JOINT " + str(jointViolated))
                else:
                    print("withinJointLimits = " + str(withinLimits))
            
            return withinLimits


def callback(data):
    # print("called")
    global base_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    base_T_PSM_SUJ = frame

if __name__ == "__main__":
    

    PSMmodel = PSMmanipulator()
    PSMmodel.LoadRobot("dvpsm.rob")
    PSMmodel.loadTool("PROGRASP_FORCEPS_420093")


    pose = np.zeros(shape=(4,4))
    joints = np.zeros(6)
    fwd = PSMmodel.ForwardKinematics(joints)

    joints,solverError = PSMmodel.InverseKinematics(joints,pose, 1e-12,1000)
    print(joints)
    rospy.init_node("AnyName")
    rospy.Rate(10000)

    import dvrk
    ARM = dvrk.psm("PSM3")
    #ECM = dvrk.ecm("ECM")
    ARM.enable()
    ARM.home()

    rospy.sleep(1)

    rospy.Subscriber('PSM3/local/measured_cp', data_class= PoseStamped, callback= callback, queue_size = 1, buff_size = 1000000)
    
    #create dVRK kinematic variable
    dvrk_model = dvrkKinematics.dvrkKinematics()

    
    while(not rospy.is_shutdown()):
        rospy.sleep(1)

        #format T
        T = pm.toMatrix(base_T_PSM_SUJ)
        # T[0][3] = 1000
        # T[1][3] = 89
        #print(T)

        # #compute the inverse kinematics (the joint angles) to achieve the current position of PSM3
        # joint = dvrk_model.ik(T)
        # joint_deg = np.array(joint[0])*180/np.pi
        # joint_deg[2] = joint_deg[2] * np.pi/180
        # #joint_deg_fixed=joint_deg
        # #joint_deg[5] = joint_deg[5] + 180
        # #Fixes joint 4
        # # if joint_deg_fixed[3]> -90:
        # #     joint_deg_fixed[3] = joint_deg_fixed[3] - 90
        # # elif joint_deg_fixed[3]<= -90:
        # #     joint_deg_fixed[3] = joint_deg_fixed[3] + 270
        # print("CloseForm IK joints = " + str(joint_deg))
        # #print("Compute FK = " + str(dvrk_model.fk(np.array(joint[0]))))

        API_joints = ARM.measured_js()[0]
        API_joints_deg = np.array(API_joints)*180/np.pi
        API_joints_deg[2] = API_joints_deg[2] * np.pi/180
        with np.printoptions(precision=5, suppress=True):
            print("API Joints are " + str(np.array(API_joints_deg)))
            #print("Compute FK =  " + str(dvrk_model.fk(API_joints)))

            #dVRK API Inverse Kinematics Python Binding
            cisst_joints = API_joints
            cisst_joints, solverError  = PSMmodel.InverseKinematics(cisst_joints,T,1e-12,500)
            cisst_joints_deg = np.array(cisst_joints)*180/np.pi
            cisst_joints_deg[2] = cisst_joints_deg[2] * np.pi/180
            diff = cisst_joints_deg - API_joints_deg

            print("Cisst/saw Joints are " + str(cisst_joints_deg))
            print("diff = " + str(diff))
            # print("fwd kinematics based on API cisst/saw = " + str(PSMmodel.ForwardKinematics(API_joints)))
            # print("fwd kinematics based on cisst/saw joints = " + str(PSMmodel.ForwardKinematics(cisst_joints)))
            # print("original T = " +str(T))
            print("Solved =" + str(solverError))
            print(PSMmodel.checkJointLimits(solverError, cisst_joints,verbose= True))
            print()
        


        #PSM1.inverse_kinematics()
        with open('MotionData.csv','a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(diff.tolist()+[""]+cisst_joints_deg.tolist()+[""]+API_joints_deg.tolist())
            #writer_object.writerow(str([str(diff[0]),str(diff[1],diff[2],diff[3],diff[4],diff[5]]))
            file_object.close()
    
        #rospy.sleep(0.5)

  
