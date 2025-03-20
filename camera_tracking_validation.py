import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import tkinter as tk
import threading
import os
import yaml
import Datalogger_CentroidTracking
from datetime import datetime


VALIDATION_DATA_ROOT="CentroidTrackingValidationData"

RightExternal_Topic='raspicam_node_right/image/compressed'
LeftExternal_Topic='raspicam_node_left/image/compressed'

#Save Directory
CALIBRATION_DIR='CameraCalibration/Calib/Calib_ForCentroidValid' #Where we store calibration parameters and images

RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"


SIZE_REDUCTION_PERCENTAGE_ECM=0.365 #Amount that external frame is reduced from original
SIZE_REDUCTION_PERCENTAGE_EXTERNAL=0.4
EXTERNAL_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*1280))
EXTERNAL_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*960))


#Detection Constants
ARUCO_IDs=[0,1,2,3]
ARUCO_SIDELENGTH=0.007
ARUCO_POINTS_3D=np.array([
    [0.0,0.0,0.0],
    [ARUCO_SIDELENGTH,0.0,0.0],
    [ARUCO_SIDELENGTH,-ARUCO_SIDELENGTH,0.0],
    [0.0,-ARUCO_SIDELENGTH,0.0]
],dtype='float32')
PERIMETER_THRESHOLD=10 #Perimeter of the marker must be larger than this in pixels
RANSAC_SCENE_REPROJECTION_ERROR=0.001
RANSAC_SCENE_ITERATIONS=400

aruco0_T_ring=np.identity(4)
aruco0_T_ring[0:4,3]=np.array([-0.005190231,0.01334,-0.006124089,1.0],dtype='float32')
#aruco0_T_ring=np.array([-0.005190231,0.01334,-0.006124089,1.0],dtype='float32')

aruco1_T_ring=np.identity(4)
aruco1_T_ring[0:4,3]=np.array([-0.005190231,-0.02143,-0.006124089,1.0],dtype='float32')
#aruco1_T_ring=np.array([-0.005190231,-0.02143,-0.006124089,1.0],dtype='float32')

aruco2_T_ring=np.identity(4)
aruco2_T_ring[0:4,3]=np.array([-0.004899909,0.01334486,-0.006124089,1.0],dtype='float32')
#aruco2_T_ring=np.array([-0.004899909,0.01334486,-0.006124089,1.0],dtype='float32')

aruco3_T_ring=np.identity(4)
aruco3_T_ring[0:4,3]=np.array([-0.01343,0.005190231,-0.006124,1.0],dtype='float32')
#aruco3_T_ring=np.array([-0.01343,0.005190231,-0.006124,1.0],dtype='float32')

aruto_T_ring_lookup={
    "0":aruco0_T_ring,
    "1":aruco1_T_ring,
    "2":aruco2_T_ring,
    "3":aruco3_T_ring
}

MUL_MAT=np.array([
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0]

],dtype='float32')

class ExternalValidator:
    def __init__(self):

        self.bridge=CvBridge()

        rospy.Subscriber(name=RightExternal_Topic,data_class=CompressedImage,callback=self.rightExternalCallback,queue_size=1,buff_size=2**20)
        rospy.Subscriber(name=LeftExternal_Topic,data_class=CompressedImage,callback=self.leftExternalCallback,queue_size=1,buff_size=2**20)



        ###################Variables and Booleans Init

        #Frames Init
        self.external_frame_left = None
        self.external_frame_right = None 

        self.gui_window=tk.Tk()
        self.gui_window.title("External View Validation GUI")
        self.gui_window.minsize(300,100)

        #Record PC2 Data
        self.recordDataButton=tk.Button(self.gui_window,text="Record Data",command=self.recrodDataCallback)
        self.recordDataButton.grid(row=0,column=0,sticky="nsew")

        #Init Aruco Marker Things
        self.aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.aruco_params=cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize=7
        self.aruco_params.cornerRefinementMaxIterations=50
        self.aruco_params.cornerRefinementMinAccuracy=0.005
        self.aruco_params.errorCorrectionRate=0.7
        #self.aruco_params.adaptiveThreshConstant=10
        self.aruco_detector=cv2.aruco.ArucoDetector(self.aruco_dict,self.aruco_params)


        #Camera Calibration Matrices:
        self.mtx_right=None
        self.mtx_left=None
        self.dist_right=None
        self.dist_left=None

        #Root Filename
        self.rootName=None



        #################Get the Camera Calibration Params
        success=self.getCameraParams()
        if not success:
            print("Failed to find camera calibration params")
            quit()
        print("Root Name With Cam Parameters: "+self.rootName)

        #Inits the datalogger object
        self.datalogger=Datalogger_CentroidTracking.Datalogger_CentroidTracking(self.mtx_left,self.dist_left,self.mtx_right,self.dist_right)
        self.is_Record=False
        self.left_external_frame_number=0
        self.right_external_frame_number=0

        #Create a thread lock
        # self.callback_lock=threading.Lock()

        #main loop
        self.gui_window.mainloop()
        #rospy.spin()

    
    def recrodDataCallback(self):
        self.is_Record=not self.is_Record
        if self.is_Record: #We are starting recording
            
            #Init the csv
            self.datalogger.initRecording(VALIDATION_DATA_ROOT)
            print("Starting Recording File: "+str(self.datalogger.record_filename))
            #init frame numbers
            self.left_external_frame_number=0
            self.right_external_frame_number=0
        else:
            print("Stopped Recording")

    #Callbacks for video feeds:
    def rightExternalCallback(self,data):
        # with self.callback_lock:
        self.external_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #Resizes the frame
        self.external_frame_right=cv2.resize(self.external_frame_right,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)

        #Logs frame if we are showing it
        if self.is_Record and self.external_frame_right is not None:
            self.datalogger.right_video_writer.write(self.external_frame_right)
            self.right_external_frame_number+=1

        self.processAndShow(show_pose=True)

    def leftExternalCallback(self,data):
        #with self.callback_lock:
        self.external_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.external_frame_left=cv2.resize(self.external_frame_left,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)

        if self.is_Record and self.external_frame_left is not None:
            self.datalogger.left_video_writer.write(self.external_frame_left)
            self.left_external_frame_number+=1

    
    def processAndShow(self,show_pose=False):
        #cv2.imshow('AutoCam_Video_GUI',self.dummy_img)
        #print(self.sharpness_kernel_value)
        if self.external_frame_left is not None and self.external_frame_right is not None:
            frame_converted_left=self.external_frame_left
            frame_converted_right=self.external_frame_right

            #Gets the aruco marker corners
            success_left,corners_filtered_left,ids_filtered_left,frame_converted_left=self.arucoTracking(frame_converted_left,show_pose)
            success_right,corners_filtered_right,ids_filtered_right,frame_converted_right=self.arucoTracking(frame_converted_right,show_pose)

            #Gets the pose and ID 
            if success_left:
                success,leftcam_T_aruco,left_id,frame_converted_left=self.getPoseandID(corners_filtered_left,ids_filtered_left,self.mtx_left,self.dist_left,frame_converted_left,show_pose)
                #Get centroid pose left:
                if success:
                    aruco_T_ring_left=aruto_T_ring_lookup[str(left_id)] #Just a point
                    # cam_T_ring_left=self.invHomogeneousNumpy(aruco_T_leftcam)@aruco_T_ring_left #Gets centroid pose
                    # cam_T_ring_left=self.invHomogeneousNumpy(cam_T_ring_left)
                    u_left,v_left=self.projectPointImagePlane(leftcam_T_aruco,aruco_T_ring_left,self.mtx_left,self.dist_left)
                    if show_pose:
                        point_2d=tuple((int(u_left),int(v_left)))
                        frame_converted_left=cv2.circle(frame_converted_left,point_2d,radius=5,color=(0,0,255),thickness=3)
            
                else:
                    leftcam_T_aruco=None
                    aruco_T_ring_left=None
                    u_left=-1
                    v_left=-1
                    left_id=-1
            else:
                leftcam_T_aruco=None
                aruco_T_ring_left=None
                u_left=-1
                v_left=-1
                left_id=-1
                    



            if success_right:
                success,rightcam_T_aruco,right_id,frame_converted_right=self.getPoseandID(corners_filtered_right,ids_filtered_right,self.mtx_right,self.dist_right,frame_converted_right,show_pose)
                #Get centroid pose right:
                if success:
                    aruco_T_ring_right=aruto_T_ring_lookup[str(right_id)]
                    # cam_T_ring_right=self.invHomogeneousNumpy(aruco_T_rightcam)@aruco_T_ring_right #Gets centroid pose
                    # cam_T_ring_right=self.invHomogeneousNumpy(cam_T_ring_right)
                    u_right,v_right=self.projectPointImagePlane(rightcam_T_aruco,aruco_T_ring_right,self.mtx_right,self.dist_right)
                    if show_pose:
                        point_2d=tuple((int(u_right),int(v_right)))
                        frame_converted_right=cv2.circle(frame_converted_right,point_2d,radius=5,color=(0,0,255),thickness=3)

                else:
                    rightcam_T_aruco=None
                    aruco_T_ring_right=None
                    u_right=-1
                    v_right=-1
                    right_id=-1
            else:
                rightcam_T_aruco=None
                aruco_T_ring_right=None
                u_right=-1
                v_right=-1
                right_id=-1

            pc_time=datetime.now()
            pc_time=pc_time.strftime("%H:%M:%S.%f")

            #Writes to the csv
            if self.is_Record:
                self.datalogger.writeRow(pc_time,self.left_external_frame_number,left_id,leftcam_T_aruco,aruco_T_ring_left,u_left,v_left,\
                                        self.right_external_frame_number,right_id,rightcam_T_aruco,aruco_T_ring_right,u_right,v_right)

            #Showing Frames
            if show_pose:
                cv2.imshow('left external',frame_converted_left)
                cv2.imshow('right external',frame_converted_right)

                keys=cv2.waitKey(1) & 0xFF
                if keys==ord('q'):
                    print("Quit")
                    self.datalogger.left_video_writer.release()
                    self.datalogger.right_video_writer.release()
                    self.gui_window.quit()
                    quit()


    def arucoTracking(self,frame,show_frame=False):
        frame_gray=cv2.cvtColor(frame.copy(),cv2.COLOR_BGR2GRAY)
        corners,ids,_=self.aruco_detector.detectMarkers(frame_gray)
        success=False
        frame_converted=frame
        corners_filtered=[]
        ids_filtered=[]
        if ids is not None:
            
            for id,corner in zip(ids,corners):
                if id[0] in ARUCO_IDs:
                    corners_filtered.append(corner)
                    ids_filtered.append([id[0]])
            
            if len(ids_filtered)>0:
                success=True
                #Showing Aruco on image
                if show_frame:
                    ids_print=np.array(ids_filtered)
                    corners_print=np.array(corners_filtered,dtype='float32')
                    frame_converted=cv2.aruco.drawDetectedMarkers(frame_converted,corners=corners_print,ids=ids_print)

        return success,corners_filtered,ids_filtered,frame_converted
    
    def getPoseandID(self,corners_filtered,ids_filtered,mtx,dist,frame,show_pose_tracking=False):
        #corners_filtered is a list of numpy arrays, and ids_filtered is a list of the filtered ids
        success=False
        cam_T_aruco=None
        id_return=None
        frame_converted=frame

        #Find aruco marker with largest perimeter (one we use for pose estimation)
        perimeter=0
        id_selected=None
        for id,corner in zip(ids_filtered,corners_filtered):
            perim_new=self.calculateMarkerSize(corner)
            if perim_new>perimeter:
                perimeter=perim_new
                id_selected=id[0]
        if (id_selected is not None) and (perimeter>PERIMETER_THRESHOLD):
            #Get the pose of the marker
            flattened_id_list=self.flattenList(ids_filtered)
            index=flattened_id_list.index(id_selected)
            image_points=corners_filtered[index]

            #Finding the aruco pose

            
            success,rotation_vector,translation_vector=cv2.solvePnP(ARUCO_POINTS_3D,image_points,mtx,dist) #flags=cv2.SOLVEPNP_IPPE)
            # success,rotation_vector,translation_vector,_=cv2.solvePnPRansac(ARUCO_POINTS_3D,image_points,mtx,dist,\
            #                                                                  iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC)
            if success:
                cam_T_aruco=self.convertRvecTvectoHomo(rotation_vector,translation_vector)
                id_return=id_selected
                if show_pose_tracking:
                    cv2.drawFrameAxes(frame_converted,mtx,dist,rotation_vector,translation_vector,0.005)
        
        return success,cam_T_aruco,id_return,frame_converted

    
    def calculateMarkerSize(self,corners):
        #Used to find the perimeter of the marker, we use the one with the largest perimeter in pixels
        perimeter=cv2.arcLength(corners,closed=True)
        return perimeter
    def flattenList(self,xss):
        return [x for xs in xss for x in xs]
    
    def convertRvecTvectoHomo(self,rvec,tvec):
        tvec=tvec.flatten()
        #print("tvec new: "+str(tvec))
        #Input: OpenCV rvec (rotation) and tvec (translation)
        #Output: Homogeneous Transform
        Rot,_=cv2.Rodrigues(rvec)
        #print("Rotation Matrix: "+str(Rot))
        transform=np.identity(4)
        transform[0:3,0:3]=Rot
        transform[0:3,3]=tvec
        transform=self.EnforceOrthogonalityNumpy_FullTransform(transform)
        return transform

    def EnforceOrthogonalityNumpy_FullTransform(self,transform):
        transform[0:3,0:3]=self.EnforceOrthogonalityNumpy(transform[0:3,0:3])

        return transform

    def EnforceOrthogonalityNumpy(self,R):
        #Function which enforces a rotation matrix to be orthogonal
        #Input: R is a 4x numpy rotation

        #Extracting columns of rotation matrix
        x=R[:,0] 
        y=R[:,1]
        z=R[:,2]
        diff_err=np.dot(x,y)
        x_orth=x-(0.5*diff_err*y)
        y_orth=y-(0.5*diff_err*x)
        z_orth=np.cross(x_orth,y_orth)
        x_norm=0.5*(3-np.dot(x_orth,x_orth))*x_orth
        y_norm=0.5*(3-np.dot(y_orth,y_orth))*y_orth
        z_norm=0.5*(3-np.dot(z_orth,z_orth))*z_orth
        R_new=np.column_stack((x_norm,y_norm,z_norm))

        return R_new
    
    def invHomogeneousNumpy(self,transform):
        #Input: Homogeneous transform (numpy)
        #Output: Inverse of homogeneous transform (numpy)
        R=transform[0:3,0:3]

        R_trans=np.transpose(R)
        d=transform[0:3,3]
        neg_R_trans_d=-1*np.dot(R_trans,d)
        inverted_transform=np.identity(4)
        inverted_transform[0:3,0:3]=R_trans
        inverted_transform[0:3,3]=neg_R_trans_d
        return inverted_transform
    

    #Folder Processing Methods
    def getFolderName(self):
        #Gets the most recent folder where we store checkerboards
        folder_count=1
        #root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        root_directory=CALIBRATION_DIR
        if os.path.isdir(root_directory):
            self.rootName=root_directory
        else:
            print("No Calibration Directory")
            return
        #old_root=root_directory
        # if os.path.isdir(root_directory):
        #     while True:
        #         if os.path.isdir(root_directory):
        #             old_root=root_directory
        #             folder_count+=1
        #             root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        #         else:
        #             break
        #     self.rootName=old_root
        # else:
        #     print("No Calibration Directory")
        #     return
        
    def getCameraParams(self):
        self.getFolderName()    #Gets Most Recent (highest numbered) File Directory
        if self.rootName is not None:
            #Reading in camera calibration params
            with open(self.rootName+RIGHT_CAMERA_CALIB_DIR+'calibration_matrix.yaml','r') as file:
                cam_info_right=yaml.load(file)
            with open(self.rootName+LEFT_CAMERA_CALIB_DIR+'calibration_matrix.yaml','r') as file:
                cam_info_left=yaml.load(file)

            #Reads in right camera calib params
            self.mtx_right=cam_info_right['camera_matrix'] #Camera Matrix
            self.mtx_right=np.array(self.mtx_right,dtype='float32')
            self.dist_right=cam_info_right['dist_coeff']    #Distortion Coefficients
            self.dist_right=np.array(self.dist_right,dtype='float32')

            #Reads in left camera calib params
            self.mtx_left=cam_info_left['camera_matrix'] #Camera Matrix
            self.mtx_left=np.array(self.mtx_left,dtype='float32')
            self.dist_left=cam_info_left['dist_coeff']    #Distortion Coefficients
            self.dist_left=np.array(self.dist_left,dtype='float32')

            return True
        else:
            return False
        
    
    ####################Point Projection Methods##################
    def projectPointImagePlane(self,cam_T_aruco,point_ring,mtx,dist):
        #ring_T_cam is the 4x4 transform bringing points from the camera to the ring
        point_ring=point_ring[0:4,3]
        ring_point_camera=cam_T_aruco@point_ring

        #ring_point=cam_T_ring[0:3,3] #Gets [Xc,Yc,Zc]
        ring_point_camera=MUL_MAT@ring_point_camera
        proj_point=mtx@ring_point_camera #Projects to image plane
        #u=proj_point[0]/proj_point[2] #Normalize by Zc
        #v=proj_point[1]/proj_point[2]
        
        #Projects with distortion
        u,v=self.projectPointsWithDistortion(proj_point[0],proj_point[1],proj_point[2],dist,mtx)
        #print("u: "+str(u)+"v: "+str(v))
        return u,v

    def projectPointsWithDistortion(self,X,Y,Z,dist_coeffs,intrinsics):
        #print("dist coeffs: "+str(dist_coeffs))
        k1, k2, p1, p2, k3 = dist_coeffs[0]
        fx=intrinsics[0,0]
        fy=intrinsics[1,1]
        cx=intrinsics[0,2]
        cy=intrinsics[1,2]

        #True points
        X=X/Z
        Y=Y/Z

        x_prime=(X-cx)/fx
        y_prime=(Y-cy)/fy
        X=x_prime
        Y=y_prime

        r2=x_prime**2+y_prime**2

        r4=r2**2
        r6=r2**3

        radial_distortion=1+k1*r2+k2*r4+k3*r6

        disp_u=X*radial_distortion+2*p1*X*Y+p2*(r2+2*(X**2))
        disp_v=Y*radial_distortion+p1*(r2+2*(Y**2))+2*p2*X*Y
        
        
        u_dis=disp_u*fx+cx
        v_dis=disp_v*fy+cy



        return u_dis,v_dis




        




if __name__=='__main__':
    rospy.init_node("ExternalValidator",anonymous=True)
    #rospy.Rate(10000)
    video_streamer=ExternalValidator()
    if video_streamer.datalogger is not None:
        video_streamer.datalogger.left_video_writer.release()
        video_streamer.datalogger.right_video_writer.release()
    video_streamer.gui_window.quit()