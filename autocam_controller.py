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

if sys.version_info.major < 3:
    input = raw_input

### THIS CODE IS THE REFACTORED VERSION OF AUTOCAM_NOGOZONE.PY ###
class autocamController: 

    def __init__(self):
        print("Initializing arms...")
    
        rospy.init_node('dvrk_teleop', anonymous=True)

        self.psm1 = dvrk.psm("PSM1")
        self.psm3 = dvrk.psm("PSM3")
        self.ecm = dvrk.ecm("ECM")

        self.setting_arms_state(self.psm1)
        self.setting_arms_state(self.psm3)
        self.setting_arms_state(self.ecm)

        ecm_pose = self.ecm.setpoint_cp()

        self.ring_offset = 0.033 ## 1.5 cm
        self.cam_offset = 0.045 ## 4 cm
        self.df = 0.11 ## in cms
        ## HARD CODED OFFSET FOR GIVEN JOINT CONFIGURATION
        ## To get to PSM1 coordinate system, take PSM3 from measure_cp and add offset
        self.offset = self.load_and_set_calibration()

        # Find respective transforms from psm1 to ring and from psm3 to cam
        self.psm1_T_R = self.ring_transform(self.ring_offset)
        self.psm3_T_cam = self.pickup_transform(self.cam_offset)    
        self.ecm_T_w = self.ecm.setpoint_cp().Inverse()

        self.message_rate = 0.01

        ## For first iteration, we need to gracefully park PSM3 at the start of our tracking...
        # Query the poses for psm1, psm3 and ecm
        self.psm1_pose = self.psm1.measured_cp()



        self.psm3_pose = self.psm3.measured_cp()

        self.ecm_T_R = self.psm1_pose * self.psm1_T_R

        z_i = pm.toMatrix(self.psm3_pose)[0:3, 2]

        self.ecm_T_psm3_desired_Pose, prev_sgn = self.orient_camera(self.psm3_T_cam, self.ecm_T_R, self.ecm_T_w, z_i, self.df, self.offset)

                #load points for noGoZone
        self.points = self.load_noGoZoneCalibration()
        #set constraints on noGoZone
        self.floor_off = 0.04 #offset from floor of calibration
        
    def pickup_transform(self,cam_offset):

        psm3_T_cam = PyKDL.Frame().Identity()
        psm3_T_cam.p[2] += cam_offset

        return psm3_T_cam

    def flip(self,is_flipped):
        pub = rospy.Publisher('isflipped', Bool, queue_size=1)
        pub.publish(is_flipped)

    def ring_transform(self,ring_offset):

        psm1_T_R = PyKDL.Frame().Identity()
        psm1_T_R.p[2] += ring_offset

        return psm1_T_R

    def tip_transform(self,tip_offset):

        psm_T_tip = PyKDL.Frame().Identity()
        psm_T_tip.p[2] += tip_offset

        return psm_T_tip

    def load_and_set_calibration(self):

        psm1_calib_pose = pickle.load(open("psm1_pose.p", "rb"))
        psm3_calib_pose = pickle.load(open("psm3_pose.p", "rb"))

        err = []

        psm1_tip_offset = 2.1/100
        psm3_tip_offset = 2.8/100

        psm1_T_tip = self.tip_transform(psm1_tip_offset)
        psm3_T_tip = self.tip_transform(psm3_tip_offset)

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
    def orient_camera(self,psm3_T_cam, ecm_T_R, ecm_T_w, z_i, df, offset):

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


    def compute_intermediate(self, psm3_T_cam, ecm_T_R, ecm_T_w, df, offset):

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
    def point_in_cube(self, P, A, B, C, D, E, verbose):
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
        def point_on_correct_side(self, P, V, normal):
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
    def point_below_floor(self, P, A, B, C, D, E, floor_offset, verbose):
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
    def orientationConstraint(self,desiredPSM3pose,ecm_T_w, verbose=False):
        flag = False
        x_wrld = pm.toMatrix(ecm_T_w)[0:3,0]
        y_cam = pm.toMatrix(desiredPSM3pose)[0:3, 1]
        if np.dot(x_wrld,y_cam) > 0:
            flag = True
        if verbose:
            print("Orientation Constraint= " + str(flag))
        return flag

    #PURPOSE: Given the current PSM3 and PSM1 pose, compute the pose of PSM3 that keeps PSM3 position constant but alters its orientation such that it views the centroid of the ring held by PSM1
    def computeSecondaryPose(self,currentPSM3pose, psm3_T_cam, ecm_T_R, ecm_T_w, offset):
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
    def computeBoundaryPose(self,desiredPSM3pose, A_p1, B_p1, C_p1, D_p1, E_p1, psm3_T_cam, ecm_T_R, ecm_T_w, offset_vec):
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
    def distanceConstraint(self,desiredPose, ecm_T_R, offset, psm3_T_cam, df, verbose = False) :
        flag = False
        desiredCameraPose = desiredPose*psm3_T_cam
        dist = LA.norm(pm.toMatrix(ecm_T_R)[0:3, 3] - pm.toMatrix(desiredCameraPose)[0:3, 3] - offset)
        # print(dist)
        if  dist < df:
            flag = True
        if verbose: 
            print("Proximity Flag = ", str(flag))
        return flag

    def interpolatePose(self,desiredPose,currentPose, verbose):
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
    def load_noGoZoneCalibration(self):
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
    def disabled(self):
        return None

    ## Setting arm states. We want to "enable" and "home" the two arms. Check operating_state for the two arms.

    def setting_arms_state(self, arm):

        if arm.operating_state() == "DISABLED":
            arm.enable()
            arm.home()

    def checkConstraints():
        inNoGo = point_in_cube(pm.toMatrix(ecm_T_psm3_desired_Pose*psm3_T_cam)[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4], verbose= False)
        belowFloor = point_below_floor(pm.toMatrix(ecm_T_psm3_desired_Pose*psm3_T_cam)[0:3,3] + np.array([offset.x(),offset.y(),offset.z()]), points[0], points[1], points[2], points[3], points[4],floor_offset=floor_off, verbose= False)
        orientationFlag = orientationConstraint(ecm_T_psm3_desired_Pose,ecm_T_w, verbose=False)
        proximityFlag = distanceConstraint(ecm_T_psm3_desired_Pose, ecm_T_R, np.array([offset.x(),offset.y(),offset.z()]),psm3_T_cam= psm3_T_cam, df=0.08, verbose=False)
            
    def computeTrajectory():
        return 0 
        
if __name__ == '__main__':

    
    
    print("Parking PSM3 to starting position...")




    if (inNoGo or belowFloor or orientationFlag or proximityFlag):
        # psm3.move_cp(ecm_T_psm3_desired_Pose)
            if (orientationFlag or proximityFlag):
                ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)


            elif (inNoGo or belowFloor):
                ecm_T_psm3_secondaryPose = computeBoundaryPose(ecm_T_psm3_desired_Pose,points[0], points[1], points[2], points[3], points[4],psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                ecm_T_psm3_secondaryPose.M = point2centroid.M

            psm3.move_cp(ecm_T_psm3_secondaryPose)

    else:
        
        point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
        ecm_T_psm3_desired_Pose.M = point2centroid.M
        psm3.move_cp(ecm_T_psm3_desired_Pose)

    psm3.jaw.move_jp(np.array([0.0]))
    rospy.sleep(message_rate)



    input("    Press Enter to start autonomous tracking...")
    
    #------------------------------------------------FILTERING INITIALIZATION---------------------------------------------------
    # positionFilter = filteringUtils.CircularBuffer(size=40,num_elements=3)
    # rotationFilter = filteringUtils.CircularBuffer(size=60,num_elements=4)
    cameraPositionFilter = filteringUtils.CircularBuffer(size=110,num_elements=3)
    cameraOrientationFiler=filteringUtils.rotationBuffer(size=10,num_elements=4)

    ######                                                                                                                ######
    ###### -------------------------------------------AUTOCAM CONTROL LOOP------------------------------------------------######
    ######                                                                                                                ######

    psm1_positionFilter = filteringUtils.CircularBuffer(size=1,num_elements=3)
    psm1_orientationFilter = filteringUtils.rotationBuffer(size=1,num_elements=4)


    while not rospy.is_shutdown():

        # Query the poses for psm1, psm3 and ecm
        psm1_pose_raw = psm1.measured_cp()

        #filter the raw psm1 inputs
        pos = psm1_pose_raw.p
        psm1_positionFilter.append(np.array([pos[0], pos[1], pos[2]]))
        filtered = psm1_positionFilter.get_mean()
        psm1_pose.p = PyKDL.Vector(filtered[0], filtered[1], filtered[2])

        psm1_orientationFilter.append(psm1_pose_raw.M)
        psm1_pose.M = psm1_orientationFilter.get_mean()



        psm3_pose = psm3.measured_cp()


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
        proximityFlag = distanceConstraint(ecm_T_psm3_desired_Pose, ecm_T_R, np.array([offset.x(),offset.y(),offset.z()]),psm3_T_cam= psm3_T_cam, df=0.08, verbose=False)
        
        print("In No Go: " + str(inNoGo) + " BelowFloor: " + str(belowFloor) + " orientationFlag: " + str(orientationFlag) + " proximityFlag: " +str(proximityFlag))


        if (inNoGo or belowFloor or orientationFlag):
            rospy.sleep(message_rate)
            
            #----------------------HANDLE NO GO ZONE----------------------------------------------------

            if (orientationFlag ):
                ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)


            elif (inNoGo or belowFloor):
                ecm_T_psm3_secondaryPose = computeBoundaryPose(ecm_T_psm3_desired_Pose,points[0], points[1], points[2], points[3], points[4],psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
                ecm_T_psm3_secondaryPose.M = point2centroid.M

                if( distanceConstraint(ecm_T_psm3_secondaryPose, ecm_T_R, np.array([offset.x(),offset.y(),offset.z()]),psm3_T_cam= psm3_T_cam, df=0.08, verbose=False) ):
                    ecm_T_psm3_secondaryPose = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)

            
            #-----------------------FILTER DESIRED PSM3 (AUTOCAM) POSITION-------------------------------
            pos = ecm_T_psm3_secondaryPose.p
            cameraPositionFilter.append(np.array([pos[0], pos[1], pos[2]]))
            pos_mean = cameraPositionFilter.get_mean()
            ecm_T_psm3_secondaryPose.p = PyKDL.Vector(pos_mean[0],pos_mean[1],pos_mean[2])

            #----------------------FILTER DESIRED PSM3 ORIENTATION ----------------------------------------
          
            cameraOrientationFiler.append(ecm_T_psm3_secondaryPose.M)
            ecm_T_psm3_secondaryPose.M  = cameraOrientationFiler.get_mean()
            #----------------------END OF FILTERING----------------------
            
            psm3.move_cp(ecm_T_psm3_secondaryPose)
            

        else:
            #----------------------------FILTER DESIRED PSM3 (AUTOCAM) POSITION-----------------------------
            pos = ecm_T_psm3_desired_Pose.p
            cameraPositionFilter.append(np.array([pos[0], pos[1], pos[2]]))
            pos_mean = cameraPositionFilter.get_mean()
            ecm_T_psm3_desired_Pose.p = PyKDL.Vector(pos_mean[0],pos_mean[1],pos_mean[2])
            
            #point at centroid
            point2centroid = computeSecondaryPose(psm3_pose,psm3_T_cam, ecm_T_R, ecm_T_w, offset)
            ecm_T_psm3_desired_Pose.M = point2centroid.M
            #----------------------------FILTERING OF PSM3 Orientation---------------------------------------
          
            cameraOrientationFiler.append(ecm_T_psm3_desired_Pose.M)
            ecm_T_psm3_desired_Pose.M  = cameraOrientationFiler.get_mean()
            #-----------------------------END OF FILTERING -------------------------------------------------------

            psm3.move_cp(ecm_T_psm3_desired_Pose)


        psm3.jaw.move_jp(np.array([0.0]))            
        rospy.sleep(message_rate)

        
            




## Initial Starting Position of PSM1: -4.634, -2.615, 240.000, 59.881, 13.880, -66.092   Jaw: 10.000