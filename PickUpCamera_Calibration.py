'''
Title: Camera Calibration and Undistortion for Autocam

'''

#Imports
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import Tkinter as tk
import os
import yaml

##########Init Constants##########

#Chekerboard Params
CHECKERBOARD_DIM=(9,10) #Number of inner corners in the checkerboard (corners height, corners width)
CHECKER_WIDTH=1.3/100 #Width of each checker in the checkerboard (in meters)
REQUIRED_CHECKERBOARD_NUM=10 #Number of checkerboard images needed for the calibration

#ROS Topics
RightExternal_Topic='raspicam_node_right/image/compressed'
LeftExternal_Topic='raspicam_node_left/image/compressed'

#Frame Resize Constants
SIZE_REDUCTION_PERCENTAGE_ECM=0.365 #Amount that external frame is reduced from original
SIZE_REDUCTION_PERCENTAGE_EXTERNAL=0.4
EXTERNAL_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*1280))
EXTERNAL_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*960))

#Save Directory
CALIBRATION_DIR='CameraCalibration/Calib/' #Where we store calibration parameters and images

RIGHT_FRAMES_FILEDIR='/chessboard_images_right/'
LEFT_FRAMES_FILEDIR='/chessboard_images_left/'

RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"


class PickUpCalibration:
    def __init__(self):

        self.bridge=CvBridge()
        #Root Filename
        self.rootName=None

        #Counters
        self.ranOnce=0
        self.frame_number=0

        #Subscribes to topics
        rospy.Subscriber(name=RightExternal_Topic,data_class=CompressedImage,callback=self.rightExternalCallback,queue_size=1,buff_size=2**20)
        rospy.Subscriber(name=LeftExternal_Topic,data_class=CompressedImage,callback=self.leftExternalCallback,queue_size=1,buff_size=2**20)

        #Frames Init
        self.external_frame_left = None
        self.external_frame_right = None 


        #Set Up GUI
        self.gui_window=tk.Tk()
        self.gui_window.title("AutoCam Video GUI")
        self.gui_window.minsize(300,100)

        #Button to grab frame
        self.button_frame=tk.Button(text="Grab Frame",width=15,command=self.grabFrameCallback)
        self.button_frame.grid(row=0,column=0,sticky="nsew")

        self.button_calibrate=tk.Button(text="Calibrate Cameras",width=15,command=self.calibrateCameraCallback)
        self.button_calibrate.grid(row=1,column=0,sticky="nsew")

        self.calibration_message_label=tk.Label(text="",width=60)
        self.calibration_message_label.grid(row=2,column=0)


        #Checkerboard subpix criteria:
        self.criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.001)

        self.mtx_right=None
        self.mtx_left=None
        self.dst_right=None
        self.dst_left=None

        #Runs the loop
        rospy.sleep(1)

        self.gui_window.mainloop()

    def showFramesCallback(self):
        if self.external_frame_left is not None:
            frameLeft=cv2.resize(self.external_frame_left,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)
            frameRight=cv2.resize(self.external_frame_right,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)
            #Showing Frames
            cv2.imshow('left',frameLeft)
            cv2.imshow('right',frameRight)
            cv2.waitKey(1) 
   
    def leftExternalCallback(self,data):
        self.external_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def rightExternalCallback(self,data):
        self.external_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.showFramesCallback()

    
    def getFolderName(self):
        #Gets the most recent folder where we store checkerboards
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        old_root=root_directory
        if os.path.isdir(root_directory):
            while True:
                if os.path.isdir(root_directory):
                    old_root=root_directory
                    folder_count+=1
                    root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
                else:
                    break
            self.rootName=old_root
        else:
            self.calibration_message_label.config(text="No Available Checkerboards, grab some")
            return
    def createSaveFolder(self):
        #Creates Root Folder to Store New Checkerboard Images
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        while True:
            if os.path.isdir(root_directory):
                folder_count+=1
                root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)                    
            else:
                break
        os.mkdir(root_directory)
        self.rootName=root_directory

    def grabFrameCallback(self):
        #Allows Frame Grabbing
        # h,w=self.external_frame_left.shape[:2]
        # print("h: "+str(h)+" w: "+str(w))
        #Resizes the images
        frameLeft=cv2.resize(self.external_frame_left,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)
        frameRight=cv2.resize(self.external_frame_right,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)
        if self.frame_number==0:
            #Initialize the save folder
            self.createSaveFolder()
        file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
        file_name_left=self.rootName+LEFT_FRAMES_FILEDIR  
        if not os.path.isdir(file_name_right):
            os.mkdir(file_name_right)
            os.mkdir(file_name_left)

        cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",frameRight)
        cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",frameLeft)

        self.frame_number+=1
        self.calibration_message_label.config(text="Frame Count: "+str(self.frame_number))

    def calibrateCameraCallback(self):
        self.getFolderName()    #Gets Most Recent (highest numbered) File Directory
        print("Root Name: "+self.rootName)
        frames_right_path=self.rootName+RIGHT_FRAMES_FILEDIR
        print("frames_right_path: "+frames_right_path)
        frames_left_path=self.rootName+LEFT_FRAMES_FILEDIR
        print("frames_left_path: "+frames_left_path)
        calibration_params_right=self.rootName+RIGHT_CAMERA_CALIB_DIR
        print("calibration_params_right: "+calibration_params_right)
        calibration_params_left=self.rootName+LEFT_CAMERA_CALIB_DIR
        print("calibration_params_left: "+calibration_params_left)

        frames_path=[frames_right_path,frames_left_path]
        params_path=[calibration_params_right,calibration_params_left]

        #################Camera Calibration###############
        #Repeats Twice, right first then left
        image_points_right=[]
        image_points_left=[]
        keeplist_right=[]
        keeplist_left=[]
        for i in range(2):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]

            if not os.path.isdir(calibration_params_path):
                os.mkdir(calibration_params_path)

            #Camera Calibration params
            objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
            objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*CHECKER_WIDTH  #I think this is already the object point size as it should be

            objpoints=[]
            imgpoints=[]

            found=0
            for file in os.listdir(checkerboard_frames_path):
                path=checkerboard_frames_path+file
                img = cv2.imread(path) # Capture frame-by-frame
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIM, None)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    if i==0:
                        keeplist_right.append(found)
                    else:
                        if not (found in keeplist_right):
                            break
                        keeplist_left.append(found)
                    objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
                    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
                    imgpoints.append(corners2)
                    print("Found Chessboard")
                    # Draw and display the corners
                    #img = cv2.drawChessboardCorners(img, CHECKERBOARD_DIM, corners2, ret)
                    #print("Drew Chessboard")
                    found += 1
                    #cv2.imshow('Chessboard', img)
                    #cv2.waitKey(1)
                    #print("Drew Image")
            
            if found>=REQUIRED_CHECKERBOARD_NUM:
                cv2.destroyWindow('Chessboard Frame')
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
                if i==0:
                    #Right params
                    self.mtx_right=mtx
                    self.dst_right=dist
                    image_points_right=imgpoints

                else:
                    self.mtx_left=mtx
                    self.dst_left=dist
                    image_points_left=imgpoints

            else:
                self.calibration_message_label.config(text="Note enough acceptable checkerboards, grab more frames, need "+str(REQUIRED_CHECKERBOARD_NUM))
                return
            

            #Finding Re-projection error
            #mean_error = 0
            #for j in range(len(objpoints)):
            #    imgpoints2, _ = cv2.projectPoints(objpoints[j], rvecs[j], tvecs[j], mtx, dist)
             #   error = cv2.norm(imgpoints[j], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
              #  mean_error += error
            #data = {'camera_matrix': np.asarray(mtx).tolist(),
                        #'dist_coeff': np.asarray(dist).tolist(),
                        #'mean reprojection error': [mean_error/len(objpoints)]}
            
            #Writing data to file
            #with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
             #   yaml.dump(data,f)
        #Cleaning up for points match
        #mismatch_indices=[k for k in range(len(keeplist_left)) if keeplist_right[i]!=keeplist_left[i]]
        #keeplist_right=[value for index, value in enumerate(keeplist_right) if index not in mismatch_indices]
        matching_indices=[k for k in range(len(keeplist_left)) if keeplist_right[k]==keeplist_left[k]]

        image_points_right=[image_points_right[keeplist_right[k]] for k in matching_indices]
        image_points_left=[image_points_left[keeplist_left[k]] for k in matching_indices]

        #Running Stereo Calibration
        stereo_calib_criteria=(cv2.TERM_CRITERIA_MAX_ITER+cv2.TERM_CRITERIA_EPS,100,1e-5)

        flags=0
        # flags|=cv2.CALIB_USE_INTRINSIC_GUESS
        # flags|=cv2.CALIB_FIX_ASPECT_RATIO
        # flags|=cv2.CALIB_ZERO_TANGENT_DIST
        # flags|=cv2.CALIB_SAME_FOCAL_LENGTH
        flags|=cv2.CALIB_FIX_INTRINSIC

        print("Done Mono calib, start stereo")
        ret,self.mtx_right,self.dst_right,self.mtx_left,self.dst_left,_,_,_,_=cv2.stereoCalibrate(objpoints,image_points_right,image_points_left,self.mtx_right,self.dst_right,self.mtx_left,self.dst_left,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT), criteria=stereo_calib_criteria,flags=flags)
                                                                                           # None,None,None,None,cv2.CALIB_FIX_ASPECT_RATIO+cv2.CALIB_ZERO_TANGENT_DIST+cv2.CALIB_USE_INTRINSIC_GUESS+cv2.CALIB_SAME_FOCAL_LENGTH+cv2.CALIB_RATIONAL_MODEL,stereo_calib_criteria)

        for i in range(2):
            calibration_params_path=params_path[i]
            if i==0:
                data = {'camera_matrix': np.asarray(self.mtx_right).tolist(),
                        'dist_coeff': np.asarray(self.dst_right).tolist()}
                with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
                    yaml.dump(data,f)
            else:
                data = {'camera_matrix': np.asarray(self.mtx_left).tolist(),
                        'dist_coeff': np.asarray(self.dst_left).tolist()}
                with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
                    yaml.dump(data,f)
                
                

            
            

        print("Finished Camera Calibration")
        self.calibration_message_label.config(text="Finished Camera Calibration")

class PickUpUndistortion:
    def __init__(self):
        #Camera Matrices, used when calling methods directly during autocam to undistort images
        self.mtx_right=None
        self.mtx_left=None
        self.dst_right=None
        self.dist_left=None

        self.rootName=None

        self.mapx_left=None
        self.mapy_left=None

        self.mapx_right=None
        self.mapy_right=None
        self.roi_left=None
        self.roi_right=None
        self.roi_left_x=None
        self.roi_left_y=None
        self.roi_left_w=None
        self.roi_left_h=None
        self.roi_right_x=None
        self.roi_right_y=None
        self.roi_right_w=None
        self.roi_right_h=None
        self.newcameramtx_left=None
        self.newcameramtx_right=None


    def getFolderName(self):
        #Gets the most recent folder where we store checkerboards
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        old_root=root_directory
        if os.path.isdir(root_directory):
            while True:
                if os.path.isdir(root_directory):
                    old_root=root_directory
                    folder_count+=1
                    root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
                else:
                    break
            self.rootName=old_root
        else:
            print("No Calibration Directory")
            return

    #Methods to undistort Image (for use in real-time with autocam)
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

            #Initializes undistortion params

            #Left
            #self.newcameramtx_left,self.roi_left=cv2.getOptimalNewCameraMatrix(self.mtx_left, self.dist_left, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT), 1, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT))
            #self.roi_left_x,self.roi_left_y,self.roi_left_w,self.roi_left_h=self.roi_left

            #self.mapx_left, self.mapy_left = cv2.initUndistortRectifyMap(self.mtx_left, self.dist_left, None, self.newcameramtx_left, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT), 5)
            self.mapx_left, self.mapy_left = cv2.initUndistortRectifyMap(self.mtx_left, self.dist_left, np.eye(3), self.mtx_left, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT), cv2.CV_16SC2)


            #right
            #self.newcameramtx_right,self.roi_right=cv2.getOptimalNewCameraMatrix(self.mtx_right, self.dist_right, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT), 1, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT))
            #self.roi_right_x,self.roi_right_y,self.roi_right_w,self.roi_right_h=self.roi_right

            #self.mapx_right, self.mapy_right = cv2.initUndistortRectifyMap(self.mtx_right, self.dist_right, None, self.newcameramtx_right, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT), 5)
            self.mapx_right, self.mapy_right = cv2.initUndistortRectifyMap(self.mtx_right, self.dist_right, np.eye(3), self.mtx_right, (EXTERNAL_WIDTH,EXTERNAL_HEIGHT), cv2.CV_16SC2)
            return True
        else:
            return False


    def undistortPickupCameraFrames(self,frame,left_right):
        #Frame can either be left or right, left_right indicates
        #The frame is already resized
        if left_right=='left': #Left frame
            #Undistorts
            #dst=cv2.remap(frame, self.mapx_left, self.mapy_left, cv2.INTER_LINEAR)
            dst=cv2.remap(frame, self.mapx_left, self.mapy_left, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT) #BORDER_REFLECT
            #dst=cv2.undistort(frame,self.mtx_left,self.dist_left,None,self.newcameramtx_left)
            

            #Crops
            #dst=dst[self.roi_left_y:self.roi_left_y+self.roi_left_h,self.roi_left_x:self.roi_left_x+self.roi_left_w]
            #dst=dst[self.roi_left_y:self.roi_left_y+self.roi_left_h,self.roi_left_x:self.roi_left_x+self.roi_left_w]

        else:   #Right frame
            #Undistorts
            #dst=cv2.remap(frame, self.mapx_right, self.mapy_right, cv2.INTER_LINEAR)
            dst=cv2.remap(frame, self.mapx_right, self.mapy_right, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
            #dst=cv2.undistort(frame,self.mtx_right,self.dist_right,None,self.newcameramtx_right)
            

            #Crops
            #dst=dst[self.roi_right_y:self.roi_right_y+self.roi_right_h,self.roi_right_x:self.roi_right_x+self.roi_right_w]
            #dst=dst[self.roi_right_y:self.roi_right_y+self.roi_right_h,self.roi_right_x:self.roi_right_x+self.roi_right_w]

        #Resize the image so that it matches the correct size
        #h,w=dst.shape[:2]
        #print("h: "+str(h)+" w: "+str(w))
        #if (h!=EXTERNAL_HEIGHT) or (w!=EXTERNAL_WIDTH):
         #   dst=cv2.resize(dst,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)

        #h,w=dst.shape[:2]
        #print("h: "+str(h)+" w: "+str(w))

        return dst





if __name__ == '__main__':
    rospy.init_node('PickUpCalibration',anonymous=True)
    pickup_calibration=PickUpCalibration() #Runs the calibration