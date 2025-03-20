# from motion import PSMmodel
import PyKDL
import numpy as np
import tf_conversions.posemath as pm
from motion import dvrkKinematics
from motion import CmnUtil



class autocamCost():

    def __init__(self, kinematicsModel = "Python"):
        self.cost = 0
        self.kinematicsModel = kinematicsModel
        self.offset = np.zeros(3)
        self.jointsLowerBounds = [-4.7124,-0.9250, 0.00, -4.5379, -1.3963, -1.3963]
        self.jointsUpperBounds = [ 4.7124,0.9250, 0.24,  4.5379,  1.3963,  1.3963]
        #C++ binding for da Vinci Kinematics
        # if kinematicsModel == "C++":
        #     self.psm = PSMmodel.PSMmanipulator()
        #     self.psm.LoadRobot("motion/dvpsm.rob") #details the DH proximal 3 most proximal joints after SUJ
        #     self.psm.loadTool("PROGRASP_FORCEPS_420093")

        #Python implementation of da Vinci Kinematics 
        # if kinematicsModel == "Python":
        self.psm = dvrkKinematics.dvrkKinematics()

        #q = computed joints
        #q_des = desired joints (what is returned by IK)
        #T_des = desired cartesian pose in frame of the ECM
        #T_target = target that you want the distance threshold to apply to in frame of ECM
        #ECM_T_PSM_RCM = PSM's RCM in the frame of the ECM
        # xxxxxReg = regularization terms (weights)
        # desiredDistance = distance from the target 
        # worldFrame = ECM_T_W
        # offset = transform between psm1 and psm3 to get to PSM1 coordinate system, take PSM3 from measure_cp and add offset

        #Initialize variables for states 
        self.q_des = np.zeros(6)
        self.T_des = PyKDL.Frame()
        self.T_target = PyKDL.Frame()
        self.ECM_T_PSM_RCM = PyKDL.Frame()
        self.offset = np.zeros(3)
        self.distanceReg = 1.0
        self.orientationReg = 1.0
        self.similarityReg = 1.0
        self.desiredDistance = 0.01
        self.psm3_T_cam = PyKDL.Frame()
        self.worldFrame = PyKDL.Frame() # ECM_T_W
    
    # def testKinematics(self, q):
    #     psm_c = PSMmodel.PSMmanipulator()
    #     psm_c.LoadRobot("motion/dvpsm.rob") #details the DH proximal 3 most proximal joints after SUJ
    #     psm_c.loadTool("PROGRASP_FORCEPS_420093")

    #     psm_py = dvrkKinematics.dvrkKinematics()

    #     print("py kinematics = " + str(psm_py.fk_prograsp_420093(q)))
    #     print("C++ kinematics = " + str(psm_c.ForwardKinematics(q)))

        



    #PURPOSE: updates the initial conditions to pass to the solver
    def initializeConditions(self, q_des, T_des, T_target, worldFrame, offset, ECM_T_PSM_RCM, psm3_T_cam, distanceReg = 1.0, orientationReg = 1.0, similarityReg = 1.0, positionReg=1.0,desOrientationReg=1.0,desiredDistance = 0.01):
        #All Frames are numpy arrays
        
        self.q_des = q_des
        self.T_des = T_des
        self.T_target = T_target
        self.ECM_T_PSM_RCM = ECM_T_PSM_RCM
        self.psm3_T_cam = psm3_T_cam
        self.offset = offset
        self.distanceReg = distanceReg
        self.orientationReg = orientationReg
        self.similarityReg = similarityReg
        self.positionReg=positionReg
        self.desiredOrientationReg=desOrientationReg
        self.desiredDistance = desiredDistance
        self.worldFrame = worldFrame

    #Takes in ECM_T_PSM_RCM and the joints,q, to compute ECM_T_PSM: ECM_T_PSM_RCM * PSM_RCM_T_
    def ECM_T_PSM(self, q):
        # if self.kinematicsModel == "C++":
        #     y = np.array([x.value() for x in q], dtype=np.float64)
        #     PSM_RCM_T_PSM = self.psm.ForwardKinematics(y)
        # elif self.kinematicsModel == "Python":
        PSM_RCM_T_PSM = self.psm.fk_prograsp_420093(q)
        return self.ECM_T_PSM_RCM @ PSM_RCM_T_PSM
    
    def compareKinematics(self, dVmeasured_cp, q):
        np_measured_cp=pm.toMatrix(dVmeasured_cp)
        ECM_T_PSM=self.ECM_T_PSM(q)
        print("measuredCp = " + str(np_measured_cp) +"\n ECM_T_PSM = " + str(ECM_T_PSM) )
        angleErr = np.rad2deg(CmnUtil.angleError(ECM_T_PSM, np_measured_cp))
        positionError = np.linalg.norm(ECM_T_PSM[0:3,3] - np_measured_cp[0:3,3])
        print("Angle Error= "+str(angleErr)+" Position error= "+str(positionError))


    #Assumes T is pyKDL frame
    #Computes the distance between a current pose defined by q and ECM_T_PSM_RCM and desired pose T_des and subtracts 
    #the threshold and computes the L2 norm of this
    def distanceError(self, ECM_T_PSM): #Shouldn't there be a norm around this outer one?##############################
        return np.linalg.norm(self.T_target[0:3,3]  - ( (ECM_T_PSM @ self.psm3_T_cam)[0:3,3] + self.offset ) ) - self.desiredDistance 

    #computes the L2 norm between the computed and desired joint state
    def jointSimilarity(self, q):
        return q - self.q_des
    
    #PURPOSE: Computes the L2 angle error between the viewing vector of the camera and the vector that views the centroid 
    def centroidAngleError(self, ECM_T_PSM):
        c = self.T_target[0:3,3] - ( (ECM_T_PSM @ self.psm3_T_cam)[0:3,3] + self.offset ) 
        z = ECM_T_PSM[0:3,2]
        return 2*self.cosThetaMinusBetweenTwoVectors(c,z)
    
    def perpendicularToFloorError(self, ECM_T_PSM):
        x_computed = ECM_T_PSM[0:3,0]
        z_w =self.worldFrame[0:3,2]
        return np.abs(self.cosThetaBetweenTwoVectors(x_computed,z_w))
    
    def orientationError(self,ECM_T_PSM):
        return 0.5 * (self.perpendicularToFloorError(ECM_T_PSM) + self.centroidAngleError(ECM_T_PSM))
    
    def positionError(self,ECM_T_PSM):
        #Takes in ECM_T_PSM pose and computes pose error from desired pose
        #Translation similarity
        t_new,t_des=ECM_T_PSM[:3,3],self.T_des[:3,3]
        trans_error=np.linalg.norm(t_new-t_des)
        return trans_error
    
    def rotationError(self,ECM_T_PSM):
        RA, RB = ECM_T_PSM[:3, :3], self.T_des[:3, :3]
        rotation_diff=np.transpose(RA)@RB
        angle_error=np.linalg.norm(rotation_diff)
        return angle_error
        
    def angleBetweenTwoVectors(self, a, b):
        a = a/np.linalg.norm(a)
        b = b/np.linalg.norm(b)
        return np.arccos(np.dot(a,b))
    
    def cosThetaMinusBetweenTwoVectors(self, a, b):
        # a = a/np.linalg.norm(a)
        # b = b/np.linalg.norm(b)
        # return 1 - np.dot(a,b)
        cos_similarity=np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))
        return 1-cos_similarity
    
    def cosThetaBetweenTwoVectors(self, a, b):
        return np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))

    
    def computeCost(self, q):
        # print("Input to Cost = ", end = "")
        # print(q)
        
        #Calculates forward kinematics first to save computation time
        ECM_T_PSM=self.ECM_T_PSM(q)

        distanceCost = self.huberLoss(self.distanceError(ECM_T_PSM), delta=0.02)
        orientationCost= self.huberLoss(self.orientationError(ECM_T_PSM), delta = 0.17) #How far off it is normal from ring
        #similarityCost = self.huberLoss(np.linalg.norm(self.jointSimilarity(q)))
        positionCost=self.huberLoss(self.positionError(ECM_T_PSM), delta = 0.01)
        #desorientationCost=self.huberLoss(self.rotationError(ECM_T_PSM))
        costTerm = self.distanceReg*distanceCost + self.orientationReg*orientationCost + self.positionReg*positionCost #+ self.similarityReg*similarityCost
        #costTerm=self.positionReg*positionCost+self.desiredOrientationReg*desorientationCost
        return costTerm
    
    def costCallback(self, q):
        print("COST = " + str(self.computeCost(q)))

    def computePoseError(self, q, verbose=True):
        T = self.ECM_T_PSM(q)
        #print("Estimated Pose: "+str(T))
        angleErr = np.rad2deg(CmnUtil.angleError(self.T_des, T))
        positionError = np.linalg.norm(T[0:3,3] - self.T_des[0:3,3])
        if verbose:
            print("Position Error = " +str(positionError) + " m AngleErr = " + str(angleErr)+" deg")
        return angleErr, positionError
        #print("joint error = " + str(q - self.q_des))
        #print("q = " +str(q) + " q_des = " + str(self.q_des))

    #PURPOSE: compute position and orientation error between two homogenous transforms
    def computeArbitraryPoseError(self,q,T_1, verbose=True):
        T = self.ECM_T_PSM(q)
        # T_1 = pm.toMatrix(T_1)
        #print("Estimated Pose: "+str(T))
        angleErr = np.rad2deg(CmnUtil.angleError(T_1, T))
        positionError = np.linalg.norm(T[0:3,3] - T_1[0:3,3])
        if verbose:
            print("Position Error = " +str(positionError) + " m AngleErr = " + str(angleErr)+" deg")
        return angleErr, positionError

    def huberLoss(self,err,delta=1.0):
        #Err is error of cost term (passed to huber/L2 Norm etc.)

        if np.abs(err)<=delta:

            cost_term=0.5*(err**2)

        else:

            cost_term=delta*(np.abs(err)-0.5*delta)

        return cost_term
    
    def l2norm(self, err):
        return np.linalg.norm(err)
    
    
    


def callbackPoseStamp(data):
    # print("called")
    global base_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    base_T_PSM_SUJ = frame

def callbackTransformStamp(data):
    # print("called")
    global ECM_T_PSM_SUJ
    p = PyKDL.Vector(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    r = PyKDL.Rotation.Quaternion(data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w)
    frame = PyKDL.Frame(r,p)
    # print(frame)
    ECM_T_PSM_SUJ = frame

ECM_T_PSM_SUJ = PyKDL.Frame() 

if __name__ == "__main__":
    import rospy
    import dvrk
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import TransformStamped
    #for testing
    rospy.init_node("AnyName")
    rospy.Rate(10000)

    ARM = dvrk.psm("PSM3")
    #ECM = dvrk.ecm("ECM")
    ARM.enable()
    ARM.home()

    rospy.sleep(1)

    rospy.Subscriber('PSM3/local/measured_cp', data_class= PoseStamped, callback= callbackPoseStamp, queue_size = 1, buff_size = 1000000)
    #subscriber to obtain the pose of the RCM of PSM3 wrt the ECM 
    PSM3sub = rospy.Subscriber('SUJ/PSM3/measured_cp', data_class= TransformStamped, callback= callbackTransformStamp, queue_size = 1, buff_size = 1000000)

    rospy.sleep(1)



    q = ARM.measured_js()[0]
    q=np.array(q.tolist(),np.float64)


    ## USING THE COST FUNCTION 
    cost = autocamCost(kinematicsModel="Python")
    cost.initializeConditions(
        q_des = np.zeros(6), 
        T_des = PyKDL.Frame(), 
        T_target = PyKDL.Frame(), 
        worldFrame= PyKDL.Frame(),
        ECM_T_PSM_RCM = ECM_T_PSM_SUJ , 
        distanceReg = 1.0, 
        orientationReg = 1.0, 
        similarityReg = 1.0, 
        desiredDistance = 0.01
    )

    print("TEST RESULTS")
    # print(cost.ECM_T_PSM(q))
    # print(cost.distanceError(q))
    # print(cost.centroidAngleError(q))
    # print(cost.computeCost(q))
    cost.compareKinematics(q = q, dVmeasured_cp=ARM.measured_cp())

    exit()
    cost.testKinematics(q)


    print("END OF TEST RESULTS")


    #initialize the solver
    import dVRKMotionSolver

    motionSolver = dVRKMotionSolver.dVRKMotionSolver(
        cost_func = cost.computeCost,
        constraint_lb = cost.jointsLowerBounds,
        constraint_up = cost.jointsUpperBounds,
        n_joints = 6, 
        verbose = True,
        solver_iterations = 100, 
        solver_tolerance= 1e-8
    )
    motionSolver.prog.AddVisualizationCallback(cost.costCallback, motionSolver.q)
    success,q,optimal_cost = motionSolver.solve_joints(q)
    cost.computePoseError(q)