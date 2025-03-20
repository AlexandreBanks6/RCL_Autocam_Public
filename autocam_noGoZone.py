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
import filteringUtils
from motion.PSMmodel import PSMmanipulator
from geometry_msgs.msg import TransformStamped
import costComputation
import dVRKMotionSolver
import OptimizationDataLogger
import time
from timeit import default_timer as timer


## GLOBALS

ECM_T_PSM_SUJ = PyKDL.Frame()


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

#psm3_T_cam = offset from psm point to camera
#ecm_T_R = pose of the ring in the ECM coordinate frame
#ecm_T_w = world pose in frame of ECM 
def orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset):

    y_R = pm.toMatrix(ecm_T_R)[0:3, 1]

    y_R = y_R / LA.norm(y_R)
    z_i = z_i / LA.norm(z_i)
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    
    # sgn1 =  np.sign(np.dot(z_i, y_R)) 
    sgn2 = np.sign(np.dot(z_w, y_R))

    # print(sgn1 * sgn2)

    #this was causing a weird flipping of the camera? 
    # z_cam = - sgn2 * y_R 
    z_cam = -1 * y_R 

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

#Computes whether or not a point is within a cube defined by 5 points: A,B,C,D,E where A,B,C,D represent the base and E is an arbitrary point that defines the height of the cube
def point_in_cube(P, A, B, C, D, E, verbose):
    # Create vectors for the base
    AB = B - A
    BC = C - B
    AC = C - A
    AD = D - A
    CD = D - C
    DA = A - D
    
    # Normal vector of the base plane
    n_base = np.cross(AB, AC)
    n_base = n_base / np.linalg.norm(n_base)
    
    # Ensure the normal vector is pointing outwards (negative z direction)
    if np.dot(n_base, A) > np.dot(n_base, A + np.array([0, 0, -1])):
        n_base = -n_base
    
    # Height vector based on point E
    height_vector = np.dot((E - A), n_base) * n_base
    
    # Calculate the top face vertices
    A_top = A + height_vector
    B_top = B + height_vector
    C_top = C + height_vector
    D_top = D + height_vector
    
    # Define the normal vectors for the six faces of the cube (pointing inwards)
    normals = [
        n_base,                     # Bottom face (ABCD)
        -n_base,                    # Top face (A_top, B_top, C_top, D_top)
        np.cross(AB, height_vector), # Side face (AB, A_top, B_top, B)
        np.cross(BC, height_vector), # Side face (AC, A_top, C_top, C)
        np.cross(AD, height_vector), # Side face (AD, A_top, D_top, D)
        np.cross(CD, height_vector)            # Opposite side face
    ]
    
    # Base and top vertices
    vertices_base = [A, B, C, D]
    vertices_top = [A_top, B_top, C_top, D_top]

    width = np.linalg.norm(AB) #base dimension
    length = np.linalg.norm(AD) #base dimension
    height = np.linalg.norm(height_vector)
    #print(f"Width = {width} m, length = {length} m, height = {height} m")
    
    # Function to check if point P is on the correct side of a plane
    def point_on_correct_side(P, V, normal):
        return np.dot(P - V, normal) <= 0
    
    # Check bottom face (ABCD)
    for vertex in vertices_base:
        if not point_on_correct_side(P, vertex, -1*normals[0]):
            if verbose:
                print("wrong side of base")
            return False
    
    # Check top face (A_top, B_top, C_top, D_top)
    for vertex in vertices_top:
        if not point_on_correct_side(P, vertex, -1*normals[1]):
            if verbose:
                print("wrong side of top")
            return False
    
    # Check side faces
    for i in range(4):
        if not point_on_correct_side(P, vertices_base[i], -1*normals[i + 2]):
            if verbose:
                print("wrong side of the sides " + str(i+1))
            return False
    
    return True

#Computes whether or not a point is below a floor defined by 5 points: A,B,C,D,E where A,B,C,D represent the base and E is an arbitrary point that defines the height of the cube
#flooroffset is the distance in meters that the "floor" is offset from face A,B,C,D along the height vector
def point_below_floor(P, A, B, C, D, E, floor_offset, verbose):
        # Create vectors for the base
    AB = B - A
    BC = C - B
    AC = C - A
    AD = D - A
    CD = D - C
    DA = A - D
    
    # Normal vector of the base plane (points inwards to the cube)
    n_base = np.cross(AC, AB) 
    n_base = n_base / np.linalg.norm(n_base)
    
    # Ensure the normal vector is pointing outwards (negative z direction)
    # if np.dot(n_base, A) > np.dot(n_base, A + np.array([0, 0, -1])):
    #     n_base = -n_base
    
    # Height vector based on point E
    height_vector = np.dot((E - A), n_base) * n_base
    
    # Calculate the top face vertices
    A_top = A + height_vector
    B_top = B + height_vector
    C_top = C + height_vector
    D_top = D + height_vector
    
    # Define the normal vectors for the six faces of the cube (pointing inwards)
    normals = [
        n_base,                     # Bottom face (ABCD)
        -n_base,                    # Top face (A_top, B_top, C_top, D_top)
        np.cross(AB, height_vector), # Side face (AB, A_top, B_top, B)
        np.cross(BC, height_vector), # Side face (B, C, B_top, C_top)
        np.cross(CD, height_vector),            # Opposite side face
        np.cross(DA, height_vector) # Side face (AD, A_top, D_top, D)

    ]
    
    # Base and top vertices
    vertices_base = [A, B, C, D]
    vertices_top = [A_top, B_top, C_top, D_top]
    vertices_sides = [[A,B,A_top, B_top],[B,C,B_top,C_top],[C,D,C_top,D_top],[A,D,D_top,A_top]]

    width = np.linalg.norm(AB) #base dimension
    length = np.linalg.norm(AD) #base dimension
    height = np.linalg.norm(height_vector)
    #print(f"Width = {width} m, length = {length} m, height = {height} m")
    
    # Function to check if point P is on the correct side of a plane
    def point_on_correct_side(P, V, normal):
        return np.dot(P - V, normal) >= 0
    
    # Check bottom face (ABCD)
    for vertex in vertices_base:
        #note that we add the height vector to the vertices because we want to account for the size of the camera
        if not point_on_correct_side(P, vertex + floor_offset*height_vector/np.linalg.norm(height_vector),normals[0]):
            if verbose:
                print("wrong side of base")
            return True
    if verbose:
        print("above base")
    
    return False

#PURPOSE: For constraining the camera to stay on one side of the robot only. Useful in the absence of inverse kinematics
def orientationConstraint(desiredPSM3pose,ecm_T_w, verbose=False):
    flag = False
    x_wrld = pm.toMatrix(ecm_T_w)[0:3,0]
    y_cam = pm.toMatrix(desiredPSM3pose)[0:3, 1]
    if np.dot(x_wrld,y_cam) > 0:
        flag = True
    if verbose:
        print("Orientation Constraint= " + str(flag))
    return flag

#PURPOSE: Given the current PSM3 and PSM1 pose, compute the pose of PSM3 that keeps PSM3 position constant but alters its orientation such that it views the centroid of the ring held by PSM1
def computeSecondaryPose(currentPSM3pose, psm3_T_cam, ecm_T_R, ecm_T_w, offset):
    #keep position the same but change the rotation

    #compute vector pointing at ring from current position
    ecm_T_cam = currentPSM3pose*psm3_T_cam
    z_psm3 = (pm.toMatrix(ecm_T_R)[0:3, 3] - pm.toMatrix(ecm_T_cam)[0:3, 3] - np.array([offset.x(),offset.y(),offset.z()]) ) / LA.norm((pm.toMatrix(ecm_T_R)[0:3, 3] - pm.toMatrix(ecm_T_cam)[0:3, 3]) - np.array([offset.x(),offset.y(),offset.z()]) )
    #compute y vector
    z_w = pm.toMatrix(ecm_T_w)[0:3, 2] / LA.norm(pm.toMatrix(ecm_T_w)[0:3, 2])
    x_psm3 = -1 * np.cross(z_psm3,z_w)/LA.norm(np.cross(z_psm3,z_w))
    #compute x vector
    y_psm3 = np.cross(z_psm3, x_psm3)

    x_cam_vec = PyKDL.Vector(x_psm3[0], x_psm3[1], x_psm3[2])
    y_cam_vec = PyKDL.Vector(y_psm3[0], y_psm3[1], y_psm3[2])
    z_cam_vec = PyKDL.Vector(z_psm3[0], z_psm3[1], z_psm3[2])
    ecm_R_psm3 = PyKDL.Rotation(x_cam_vec, y_cam_vec, z_cam_vec)
    

    secondaryPose = PyKDL.Frame(ecm_R_psm3,currentPSM3pose.p)

    return secondaryPose

#Computes the pose that is closest to the desired point but lies on the top boundary of the noGoZone. A_p1 etc are the nogo zone defined in PSM1 coordinate system
def computeBoundaryPose(desiredPSM3pose, A_p1, B_p1, C_p1, D_p1, E_p1, psm3_T_cam, ecm_T_R, ecm_T_w, offset_vec):
        offset = np.array([offset_vec.x(),offset_vec.y(),offset_vec.z()])
        
        
        #Convert nogozone into psm3 coordinate system
        A = A_p1 - offset
        B = B_p1 - offset
        C = C_p1 - offset
        D = D_p1 - offset
        E = E_p1 - offset


        AB = B - A
        BC = C - B
        AC = C - A
        AD = D - A
        CD = D - C
        DA = A - D
        
        # Normal vector of the base plane (points inwards to the cube)
        n_base = np.cross(AC, AB) 
        n_base = n_base / np.linalg.norm(n_base)
        
        # Height vector based on point E
        height_vector = np.dot((E - A), n_base) * n_base
        
        # Calculate the top face vertices
        A_top = A + height_vector
        B_top = B + height_vector
        C_top = C + height_vector
        D_top = D + height_vector
        
        # Define the normal vectors for the six faces of the cube (pointing inwards)
        normals = [
            n_base,                     # Bottom face (ABCD)
            -n_base,                    # Top face (A_top, B_top, C_top, D_top)
            np.cross(AB, height_vector), # Side face (AB, A_top, B_top, B)
            np.cross(BC, height_vector), # Side face (B, C, B_top, C_top)
            np.cross(CD, height_vector),            # Opposite side face
            np.cross(DA, height_vector) # Side face (AD, A_top, D_top, D)
        ]

        #compute vector for offsetting point to outside of the noGoZone
        desiredPosition = pm.toMatrix(desiredPSM3pose)[0:3,3]
        #vector to the plane
        vec2plane = -1 * np.dot(desiredPosition - A_top, normals[1]) * normals[1]

        compensatedPosition = desiredPosition + vec2plane

        #enforce view distance

        # z = (compensatedPosition - pm.toMatrix(ecm_T_R)[0:3, 3]  - offset ) / LA.norm(compensatedPosition - pm.toMatrix(ecm_T_R)[0:3, 3]  - offset )

        # enforcedPosition = 1 * z * 0.12 + (pm.toMatrix(ecm_T_R)[0:3, 3] - offset)
        # enforcedPosition = PyKDL.Vector(enforcedPosition[0], enforcedPosition[1], enforcedPosition[2])

        desiredPSM3pose.p = PyKDL.Vector(compensatedPosition[0], compensatedPosition[1], compensatedPosition[2])

        # secondaryPose = computeSecondaryPose(desiredPSM3pose, psm3_T_cam, ecm_T_R, ecm_T_w, offset_vec)
        return desiredPSM3pose

#PURPOSE: To monitor proximity of arm to ring 
def distanceConstraint(desiredPose, ecm_T_R, offset, psm3_T_cam, df, verbose = False) :
    flag = False
    desiredCameraPose = desiredPose*psm3_T_cam
    dist = LA.norm(pm.toMatrix(ecm_T_R)[0:3, 3] - pm.toMatrix(desiredCameraPose)[0:3, 3] - offset)
    # print(dist)
    if  dist < df:
        flag = True
    if verbose: 
        print("Proximity Flag = ", str(flag))
    return flag





def interpolatePose(desiredPose,currentPose, verbose):
    interpolationRequired = False

    #check distance between desired and current pose 
    displacement = pm.toMatrix(desiredPose)[0:3, 3] - pm.toMatrix(currentPose)[0:3, 3]
    distance = LA.norm(displacement)
    direction = displacement/distance
    intermediatePose = PyKDL.Frame()

    if distance > 0.015 : 
        interpolationRequired = True
        intermediatePosition = pm.toMatrix(currentPose)[0:3, 3] + direction*distance*0.5
        intermediateP =PyKDL.Vector(intermediatePosition[0], intermediatePosition[1], intermediatePosition[2])
        intermediateRotation = desiredPose.M
        intermediatePose = PyKDL.Frame(intermediateRotation, intermediateP)

    if verbose:
        print("Interpolation Flag = " + str(interpolationRequired))
    return [interpolationRequired, intermediatePose]

#loads the noGoZone calibrations. Returns a point array of the noGoZone in ***tip space***
def load_noGoZoneCalibration():
    psm1_tip_offset = 2.1/100
    psm3_tip_offset = 2.8/100
    psm1_T_tip = tip_transform(psm1_tip_offset)


    psm1_calib_pose = pickle.load(open("psm1_pose_noGo.p", "rb"))
    points = []
    for p in range(len(psm1_calib_pose)):
        ecm_T_psm1_tip = psm1_calib_pose[p]*psm1_T_tip
        points.append(pm.toMatrix(ecm_T_psm1_tip)[0:3, 3])
    return points

## When the state is distabled, then don't run teloperation
def disabled():
    return None

## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

def setting_arms_state(arm):

    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()
        
        #callback for obtaining PSM3 RCM wrt the ECM
def callback(data):
    # print("called")
    global ECM_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    ECM_T_PSM_SUJ = frame


if __name__ == '__main__':

    print("Initializing arms...")
    
   #configure psm kinematic model and solver 
    PSMmodel = PSMmanipulator()
    PSMmodel.LoadRobot("motion/dvpsm.rob") #details the DH proximal 3 most proximal joints after SUJ
    PSMmodel.loadTool("PROGRASP_FORCEPS_420093")

    rospy.init_node('dvrk_teleop', anonymous=True)
    rospy.Rate(10000)


    psm1 = dvrk.psm("PSM1")
    psm3 = dvrk.psm("PSM3")
    ecm = dvrk.ecm("ECM")

    setting_arms_state(psm1)
    setting_arms_state(psm3)
    setting_arms_state(ecm)
    
    rospy.sleep(1)
 
    #subscriber to obtain the pose of the RCM of PSM3 wrt the ECM 
    PSM3sub = rospy.Subscriber('SUJ/PSM3/measured_cp', data_class= TransformStamped, callback= callback, queue_size = 1, buff_size = 1000000)


    ecm_pose = ecm.setpoint_cp()

    ring_offset = 0.033 ## 1.5 cm
    cam_offset = 0.045 ## 4 cm
    df = 0.11 ## in cms
    ## HARD CODED OFFSET FOR GIVEN JOINT CONFIGURATION
    ## To get to PSM1 coordinate system, take PSM3 from measure_cp and add offset
    offset = load_and_set_calibration()

    # Find respective transforms from psm1 to ring and from psm3 to cam
    psm1_T_R = ring_transform(ring_offset)
    psm3_T_cam = pickup_transform(cam_offset)    
    ecm_T_w = ecm.setpoint_cp().Inverse()

    message_rate = 0.005

    ## For first iteration, we need to gracefully park PSM3 at the start of our tracking...
    # Query the poses for psm1, psm3 and ecm
    psm1_pose = psm1.measured_cp()
    psm3_pose = psm3.measured_cp()
    ecm_T_R = psm1_pose * psm1_T_R
    z_i = pm.toMatrix(psm3_pose)[0:3, 2]
    ecm_T_psm3_desired_Pose, prev_sgn = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)
    

      #load points for noGoZone
    points = load_noGoZoneCalibration()
    #set constraints on noGoZone
    floor_off = 0.04 #offset from floor of calibration

    #-----------------------------SOLVER INITIALIZATION----------------------------------
    cost = costComputation.autocamCost(kinematicsModel="Python")


        #initialize the solver
    motionSolver = dVRKMotionSolver.dVRKMotionSolver(
        cost_func = cost.computeCost,
        constraint_lb = cost.jointsLowerBounds,
        constraint_up = cost.jointsUpperBounds,
        n_joints = 6, 
        verbose = False,
        solver_iterations=50,
        solver_tolerance=5e-4,
        max_solver_time=0.05, 
        solverName="NLOPT",
        solver_algorithm="LD_SLSQP"
    )
    # motionSolver.prog.AddVisualizationCallback(cost.costCallback, motionSolver.q)

    #Initializes datalogger to debug solver
    datalogger=OptimizationDataLogger.OptimizationDataLogger()
    datalogger.initRecording(offset,psm3_T_cam,points,psm1_T_R)

    OPTIMIZEPOSE = True #False = no optimization for pose when IK fails, True = optimization of pose when IK fails


    
    #------------------------------------------------FILTERING INITIALIZATION---------------------------------------------------
    # positionFilter = filteringUtils.CircularBuffer(size=40,num_elements=3)
    # rotationFilter = filteringUtils.CircularBuffer(size=60,num_elements=4)
    cameraPositionFilter = filteringUtils.CircularBuffer(size=140,num_elements=3)
    cameraOrientationFiler=filteringUtils.rotationBuffer(size=20,num_elements=4)

    ######                                                                                                                ######
    ###### -------------------------------------------AUTOCAM CONTROL LOOP------------------------------------------------######
    ######                                                                                                                ######

    psm1_positionFilter = filteringUtils.CircularBuffer(size=5,num_elements=3)
    psm1_orientationFilter = filteringUtils.rotationBuffer(size=5,num_elements=4)

    initialized = False



    while not rospy.is_shutdown():
        start_time=timer()
        # Query the poses for psm1, psm3 and ecm
        try:
            psm1_pose_raw = psm1.measured_cp()
        except Exception as e:
            print("Unable to read psm1: "+str(e))
            continue

        #filter the raw psm1 inputs
        pos = psm1_pose_raw.p
        psm1_positionFilter.append(np.array([pos[0], pos[1], pos[2]]))
        filtered = psm1_positionFilter.get_mean()
        psm1_pose.p = PyKDL.Vector(filtered[0], filtered[1], filtered[2])

        psm1_orientationFilter.append(psm1_pose_raw.M)
        psm1_pose.M = psm1_orientationFilter.get_mean()


        try:
            psm3_pose = psm3.measured_cp()
        except Exception as e:
            print("Unable to read psm3: "+str(e))
            continue



        #compute optimal desired pose
        ecm_T_R = psm1_pose * psm1_T_R
        z_i = pm.toMatrix(psm3_pose)[0:3, 2]
        ecm_T_psm3_desired_Pose, curr_sgn = orient_camera(psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset)

        #INTERPOLATE POSE
        interpolationRequired, interpolatedPose = interpolatePose(ecm_T_psm3_desired_Pose, psm3_pose, verbose= False)

        if interpolationRequired:
            ecm_T_psm3_desired_Pose = interpolatedPose


        #NOGOZONE FLAGS
        inNoGo = point_in_cube(pm.toMatrix(ecm_T_psm3_desired_Pose*psm3_T_cam)[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4], verbose= False)
        belowFloor = point_below_floor(pm.toMatrix(ecm_T_psm3_desired_Pose*psm3_T_cam)[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4],floor_offset=floor_off, verbose= False)
        orientationFlag = orientationConstraint(ecm_T_psm3_desired_Pose,ecm_T_w, verbose=False)
        #proximityFlag = distanceConstraint(ecm_T_psm3_desired_Pose, ecm_T_R, np.array([offset.x(),offset.y(),offset.z()]),psm3_T_cam= psm3_T_cam, df=0.08, verbose=False)
        
        #Initialized for data recording
        proximity_flag=False
        ecm_T_psm3_des_proximity=NOT_A_TRANSFORM
        ecm_T_psm3_des=NOT_A_TRANSFORM
        q=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        success = False #solver success 


        if (inNoGo or belowFloor or orientationFlag):
            rospy.sleep(message_rate)
            try:
                q_curr = psm3.measured_js()[0]
                jointState=q_curr

            except Exception as e:
                print("Unable to read psm3: "+str(e))
                continue
            
            #----------------------HANDLE NO GO ZONE----------------------------------------------------

            if (orientationFlag):
                ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)


            elif (inNoGo or belowFloor):
                ecm_T_psm3_secondaryPose = computeBoundaryPose(ecm_T_psm3_desired_Pose,points[0], points[1], points[2], points[3], points[4],psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                ecm_T_psm3_secondaryPose.M = point2centroid.M

                proximity_flag=distanceConstraint(ecm_T_psm3_secondaryPose, ecm_T_R, np.array([offset.x(),offset.y(),offset.z()]),psm3_T_cam= psm3_T_cam, df=0.08, verbose=False) 
                if(proximity_flag):
                    print("Distance Constraint")
                    #ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                    if OPTIMIZEPOSE:
                        cost.initializeConditions(
                            q_des = 0.0, 
                            T_des = pm.toMatrix(ecm_T_psm3_secondaryPose), 
                            T_target = pm.toMatrix(ecm_T_R), 
                            worldFrame= pm.toMatrix(ecm_T_w),
                            ECM_T_PSM_RCM = pm.toMatrix(ECM_T_PSM_SUJ) , 
                            psm3_T_cam= pm.toMatrix(psm3_T_cam),
                            offset= np.array([offset.x(), offset.y(), offset.z()]),
                            distanceReg = 25.0, 
                            orientationReg = 30.0, 
                            similarityReg = 1.0, #Not in use
                            positionReg=15.0,
                            desOrientationReg=2000.0, #Not in use
                            desiredDistance = df #meters

                        )

                        ecm_T_psm3_des_proximity=ecm_T_psm3_secondaryPose #For data recording


                        success,q,optimal_cost = motionSolver.solve_joints(q_curr)
                        print(f"Solver Success: {success}")                        

                    #Commanding to position
                    #if success:
                        ecm_T_psm3_secondaryPose=cost.ECM_T_PSM(q)
                        ecm_T_psm3_secondaryPose=pm.fromMatrix(ecm_T_psm3_secondaryPose)


                    else:

                        ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)           

            
            #check if desired pose violate IK constraints
            #inverse kinematics 
            PSM3rcm_T_PSM3 = ECM_T_PSM_SUJ.Inverse() *  ecm_T_psm3_secondaryPose
            jointState, solverError  = PSMmodel.InverseKinematics(jointState, pm.toMatrix(PSM3rcm_T_PSM3),1e-12,500)
            jointLimitFlag = PSMmodel.checkJointLimits(solverError,jointState, verbose= False)
            if not jointLimitFlag and initialized:
                #compute optimization-based pose placement
                if OPTIMIZEPOSE:
                    cost.initializeConditions(
                        q_des = jointState, 
                        T_des = pm.toMatrix(ecm_T_psm3_secondaryPose), 
                        T_target = pm.toMatrix(ecm_T_R), 
                        worldFrame= pm.toMatrix(ecm_T_w),
                        ECM_T_PSM_RCM = pm.toMatrix(ECM_T_PSM_SUJ) , 
                        psm3_T_cam= pm.toMatrix(psm3_T_cam),
                        offset= np.array([offset.x(), offset.y(), offset.z()]),
                        distanceReg = 10.0, 
                        orientationReg = 30.0, 
                        similarityReg = 1.0, #Not in use
                        positionReg=25.0,
                        desOrientationReg=2000.0, #Not in use
                        desiredDistance = df #meters

                    )
                    success,q,optimal_cost = motionSolver.solve_joints(q_curr)
                    print(f"Solver Success: {success}")

                #if success:
                    ecm_T_psm3_des=ecm_T_psm3_secondaryPose#Stored for recording

                    ecm_T_psm3_secondaryPose=cost.ECM_T_PSM(q)
                    ecm_T_psm3_secondaryPose=pm.fromMatrix(ecm_T_psm3_secondaryPose)

                else:

                    ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                
                
                print("Secondary Pose IK failed ")

            end_time=timer()
            elapsed_time=start_time-end_time
            #########################Write data row########################################
            data_row=[elapsed_time,[not jointLimitFlag,success,inNoGo,belowFloor,orientationFlag,proximity_flag],\
                pm.toMatrix(psm1_pose_raw),pm.toMatrix(psm1_pose),pm.toMatrix(psm3_pose),pm.toMatrix(ECM_T_PSM_SUJ),\
                    pm.toMatrix(ecm_T_psm3_des),pm.toMatrix(ecm_T_psm3_des_proximity),pm.toMatrix(ecm_T_psm3_secondaryPose),\
                        pm.toMatrix(ecm_T_R),pm.toMatrix(ecm_T_w),np.array(q_curr).tolist(),q]
            datalogger.writeRow(data_row)


            # if(distanceConstraint(ecm_T_psm3_secondaryPose, ecm_T_R, np.array([offset.x(),offset.y(),offset.z()]),psm3_T_cam= psm3_T_cam, df=0.08, verbose=False) ):
            #     print("Distance Constraint")
            #     ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                        #-----------------------FILTER DESIRED PSM3 (AUTOCAM) POSITION-------------------------------
            pos = ecm_T_psm3_secondaryPose.p
            cameraPositionFilter.append(np.array([pos[0], pos[1], pos[2]]))
            pos_mean = cameraPositionFilter.get_mean()
            ecm_T_psm3_secondaryPose.p = PyKDL.Vector(pos_mean[0],pos_mean[1],pos_mean[2])

            #----------------------FILTER DESIRED PSM3 ORIENTATION ----------------------------------------
            # point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
            # ecm_T_psm3_secondaryPose.M = point2centroid.M

            cameraOrientationFiler.append(ecm_T_psm3_secondaryPose.M)
            ecm_T_psm3_secondaryPose.M  = cameraOrientationFiler.get_mean()
            #----------------------END OF FILTERING----------------------

            psm3.move_cp(ecm_T_psm3_secondaryPose)
            

        else:

             #check if desired pose violate IK constraints
            #inverse kinematics 
            try:
                q_curr = psm3.measured_js()[0]
                jointState=q_curr

            except Exception as e:
                print("Unable to read psm3: "+str(e))
                continue

            PSM3rcm_T_PSM3 = ECM_T_PSM_SUJ.Inverse() *  ecm_T_psm3_desired_Pose
            jointState, solverError  = PSMmodel.InverseKinematics(jointState, pm.toMatrix(PSM3rcm_T_PSM3),1e-12,500)
            jointLimitFlag = PSMmodel.checkJointLimits(solverError,jointState, verbose= False)
            #print("joint state = " +str(jointState))
            

            if not jointLimitFlag and initialized:
                
                if OPTIMIZEPOSE:
                    #compute optimization-based pose placement
                    cost.initializeConditions(
                        q_des = jointState, 
                        T_des = pm.toMatrix(ecm_T_psm3_desired_Pose), 
                        T_target = pm.toMatrix(ecm_T_R), 
                        worldFrame= pm.toMatrix(ecm_T_w),
                        ECM_T_PSM_RCM = pm.toMatrix(ECM_T_PSM_SUJ) , 
                        psm3_T_cam= pm.toMatrix(psm3_T_cam),
                        offset= np.array([offset.x(), offset.y(), offset.z()]),
                        distanceReg = 10.0, 
                        orientationReg = 30.0, 
                        similarityReg = 1.0, #Not in use
                        positionReg=25.0,
                        desOrientationReg=2000.0, #Not in use
                        desiredDistance = df #meters
                    )

                    success, q, optimal_cost = motionSolver.solve_joints(q_curr)
                    print(f"Solver Success: {success}")
                

                # Commanding to position
                #if success:
                    ecm_T_psm3_des=ecm_T_psm3_desired_Pose#Stored for recording

                    ecm_T_psm3_desired_Pose=cost.ECM_T_PSM(q)
                    ecm_T_psm3_desired_Pose=pm.fromMatrix(ecm_T_psm3_desired_Pose)

                else:
                    ecm_T_psm3_desired_Pose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                
                
                print("Primary Pose IK failed ")


            #point at centroid
            point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
            ecm_T_psm3_desired_Pose.M = point2centroid.M


            end_time=timer()
            elapsed_time=start_time-end_time
            #########################Write data row########################################
            data_row=[elapsed_time,[not jointLimitFlag,success,inNoGo,belowFloor,orientationFlag,proximity_flag],\
                pm.toMatrix(psm1_pose_raw),pm.toMatrix(psm1_pose),pm.toMatrix(psm3_pose),pm.toMatrix(ECM_T_PSM_SUJ),\
                    pm.toMatrix(ecm_T_psm3_des),pm.toMatrix(ecm_T_psm3_des_proximity),pm.toMatrix(ecm_T_psm3_desired_Pose),\
                        pm.toMatrix(ecm_T_R),pm.toMatrix(ecm_T_w),np.array(q_curr).tolist(),q]
            datalogger.writeRow(data_row)


            #----------------------------FILTER DESIRED PSM3 (AUTOCAM) POSITION-----------------------------
            pos = ecm_T_psm3_desired_Pose.p
            cameraPositionFilter.append(np.array([pos[0], pos[1], pos[2]]))
            pos_mean = cameraPositionFilter.get_mean()
            ecm_T_psm3_desired_Pose.p = PyKDL.Vector(pos_mean[0],pos_mean[1],pos_mean[2])
            
            
            #----------------------------FILTERING OF PSM3 Orientation---------------------------------------
          
            cameraOrientationFiler.append(ecm_T_psm3_desired_Pose.M)
            ecm_T_psm3_desired_Pose.M  = cameraOrientationFiler.get_mean()
            #-----------------------------END OF FILTERING -------------------------------------------------------

            psm3.move_cp(ecm_T_psm3_desired_Pose)

        print("In No Go: " + str(inNoGo) + ", BelowFloor: " + str(belowFloor) + ", orientationFlag: " + str(orientationFlag) + ", IKtriggered = " + str(not jointLimitFlag) + ", solverSuccess = " +str(success), end = "\n\n")


        psm3.jaw.move_jp(np.array([0.0])) 
        
        if (not initialized):
            initialized = True
            print("Parking PSM3 to starting position...")
            input("    Press Enter to start autonomous tracking...")           

        rospy.sleep(message_rate)

        
            




## Initial Starting Position of PSM1: -4.634, -2.615, 240.000, 59.881, 13.880, -66.092   Jaw: 10.000