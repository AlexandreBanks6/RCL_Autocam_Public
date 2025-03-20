'''
Title: ECM + External Camera Video Streamer
Date: July 4th, 2024
Authors: Alexandre Banks & Randy Moore
Description: Takes in the left/right ECM from the dVRK and the left/right video feed 
from an external camera and superimposes the external camera below the ECM frame

Steps to Setup ROS for Cameras:

1. Connect green ethernet to USB-to-Ethernet adapter and connect to USB port, then:
    1.1 press 'Ethernet Connection' and under that select 'wired connection'
    1.2 $ssh pi@192.168.0.12
    1.3 $sudo ifconfig eth0 192.168.1.12 netmask 255.255.255.0 up

    Don't close terminal yet (not until step 8 below)!

2. Connect white ethernet to wired ethernet
    - Under 'PCI Ethernet Connected' or 'wired ethernet':
        set it to 'Wired Ethernet' 
    - Under 'Ethernet Connected' or 'USB Ethernet' set it to 'USB Ethernet'
4. (this step should already be done) $ifconfig should show 'eno1' with inet: 192.168.0.11
  and 'enx00e05a001959' with inet: 192.168.1.13 (note the names for these two ports may change)
  If these are not the IP addresses showing, configure these IP address with:
  $sudo ifconfig <ethernet port name> <desired IP from above> netmask 255.255.255.0 up
  Note: these should match what the wired ethertnet GUI has for 'USB Ethernet' and 'Wired Ethernet' profiles

5. $roscore on dVRK-pc2
6. $roslaunch dvrk_robot jhu_daVinci_video.launch rig_name:=ubc_dVRK_ECM
    - Gets the left/right ECM feeds with topics:
    'ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
    'ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
7. ssh pi@192.168.0.10 => Enters left pi
 7.2 export ROS_IP=192.168.0.10
 7.3 export ROS_MASTER_URI=http://192.168.0.11:11311
 7.4 roslaunch raspicam_node camerav2_1280x960.launch
    - Starts left external camera video feed with ros topic:
    'raspicam_node_left/image/compressed'
8. Close terminal window from step 1 where we did $ssh pi@192.168.0.12
9. ssh pi@192.168.1.12 => Enters right pi
 9.2 export ROS_IP=192.168.1.12
 9.3 export ROS_MASTER_URI=http://192.168.1.13:11311
 9.4 roslaunch raspicam_node camerav2_1280x960.launch
    - Starts right external camera video feed with ros topic:
    'raspicam_node_right/image/compressed'

Note: Password for both pi's is mech464
'''


import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import Tkinter as tk
import PickUpCamera_Calibration

#ROS Topics
RightECM_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftECM_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
RightExternal_Topic='raspicam_node_right/image/compressed'
LeftExternal_Topic='raspicam_node_left/image/compressed'
SIZE_REDUCTION_PERCENTAGE_ECM=0.365 #Amount that external frame is reduced from original
SIZE_REDUCTION_PERCENTAGE_EXTERNAL=0.4
EXTERNAL_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*1280))
EXTERNAL_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*960))

ECM_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_ECM*1400))
ECM_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_ECM*986))

VIEW_WINDOW_WIDTH=1024
VIEW_WINDOW_HEIGHT=768

EXTERNAL_SEPARATION=50 #Pixel offset from center line for left (neg offset) and right (pos offset)

ZEROS_ARRAY=np.zeros((EXTERNAL_HEIGHT,EXTERNAL_WIDTH,3))
MAX_ARRAY=255*np.ones((EXTERNAL_HEIGHT,EXTERNAL_WIDTH,3))
def nothing(x):
    pass

class ECMVideoStreamer:
    def __init__(self):


        self.bridge=CvBridge()

        #TKinter Window Setup

        #Subscribing to ROS Topics
        rospy.Subscriber(name=RightECM_Topic,data_class=CompressedImage,callback=self.rightECMCallback,queue_size=1,buff_size=2**20)
        rospy.Subscriber(name=LeftECM_Topic,data_class=CompressedImage,callback=self.leftECMCallback,queue_size=1,buff_size=2**20)
      
        
        rospy.Subscriber(name=RightExternal_Topic,data_class=CompressedImage,callback=self.rightExternalCallback,queue_size=1,buff_size=2**20)
        rospy.Subscriber(name=LeftExternal_Topic,data_class=CompressedImage,callback=self.leftExternalCallback,queue_size=1,buff_size=2**20)

        ###################Variables and Booleans Init

        #Frames Init
        self.external_frame_left = None
        self.external_frame_right = None 
        self.ecm_frame_left = None 
        self.ecm_frame_right = None


        #Variabels to change with GUI
        self.sharpness_kernel_value=15
        self.sharpness_sigma_value=2
        self.external_separation_val=EXTERNAL_SEPARATION

        #toggles external boolean
        self.is_external_on=True

        #Object to undistort frames
        self.undistortion_obj=PickUpCamera_Calibration.PickUpUndistortion()
        #Gets the camera params
        self.is_undistort_available=self.undistortion_obj.getCameraParams()
        self.is_undistort_on=True   #Toggles undistort on

        #Create slider window
        #self.dummy_img=np.zeros((200,600,3),np.uint8)
        #cv2.namedWindow('AutoCam_Video_GUI')
        #cv2.createTrackbar('Sharpness Kernel','AutoCam_Video_GUI',0,255,nothing)
        self.gui_window=tk.Tk()
        self.gui_window.title("AutoCam Video GUI")
        self.gui_window.minsize(300,100)
        #self.gui_window.gri
        #self.gui_window.rowconfigure([0,1,2],weight=1)
        #self.gui_window.columnconfigure([0,1],weight=1)

        #Kernel Value for Sharpening
        self.kernel_label=tk.Label(self.gui_window,text='Sharpness Kernel: ').grid(row=0,column=0,sticky="nsew")
        self.slider_kernel=tk.Scale(self.gui_window,from_=0,to=25,orient="horizontal")
        self.slider_kernel.set(self.sharpness_kernel_value)
        self.slider_kernel.bind("<ButtonRelease-1>",self.kernelTrackbarCallback)
        self.slider_kernel.grid(row=0,column=1,sticky="nsew")

        #Sigma Value for Sharpening
        self.sigma_label=tk.Label(self.gui_window,text='Sharpness Sigma: ').grid(row=1,column=0,sticky="nsew")
        self.slider_sigma=tk.Scale(self.gui_window,from_=0,to=5,orient="horizontal")
        self.slider_sigma.set(self.sharpness_sigma_value)
        self.slider_sigma.bind("<ButtonRelease-1>",self.sigmaTrackbarCallback)
        self.slider_sigma.grid(row=1,column=1,sticky="nsew")

        #Kernel Value for Sharpening
        self.separation_label=tk.Label(self.gui_window,text='External Cam. Separation: ').grid(row=2,column=0,sticky="nsew")
        self.slider_separation=tk.Scale(self.gui_window,from_=-100,to=100,orient="horizontal")

        self.slider_separation.set(self.external_separation_val)
        self.slider_separation.bind("<ButtonRelease-1>",self.separationTrackbarCallback)
        self.slider_separation.grid(row=2,column=1,sticky="nsew")

        #Button to turn on/off undistortion (default is on)
        self.toggle_undistortion_button=tk.Button(self.gui_window,text="Toggle Undistortion (default on)",command=self.toggleUndistortion)
        self.toggle_undistortion_button.grid(row=3,column=0,sticky='nsew')

        #Button to turn on/off external camera view
        self.toggle_external_button=tk.Button(self.gui_window,text="Toggle External Camera",command=self.toggleExternalCallback)
        self.toggle_external_button.grid(row=4,column=0,sticky='nsew')


        #main loop
        self.gui_window.mainloop()
        #rospy.spin()
    
    def kernelTrackbarCallback(self,event):
        self.sharpness_kernel_value=self.slider_kernel.get()
        if (self.sharpness_kernel_value//2*2)==self.sharpness_kernel_value: #Fixes Kernel to an odd value
            self.sharpness_kernel_value=self.sharpness_kernel_value+1
        
    def sigmaTrackbarCallback(self,event):
        self.sharpness_sigma_value=self.slider_sigma.get()

    def separationTrackbarCallback(self,event):
        self.external_separation_val=self.slider_separation.get()

    def toggleExternalCallback(self):
        self.is_external_on=not self.is_external_on

    def toggleUndistortion(self):
        self.is_undistort_on=not self.is_undistort_on
        print("Undistortion On: "+str(self.is_undistort_on))


    def showFramesCallback(self):
        #cv2.imshow('AutoCam_Video_GUI',self.dummy_img)
        #print(self.sharpness_kernel_value)
        if self.ecm_frame_left is not None:

            superimposed_left=self.superimposeView(self.ecm_frame_left,self.external_frame_left,'left')
            superimposed_right=self.superimposeView(self.ecm_frame_right,self.external_frame_right,'right')
            #Showing Frames
            cv2.imshow('left_eye',superimposed_left)
            cv2.imshow('right_eye',superimposed_right)
            cv2.imshow('left_eye_desktop_view',superimposed_left)
            cv2.imshow('right_eye_desktop_view',superimposed_right)
            keys=cv2.waitKey(1) & 0xFF
            if keys==ord('q'):
                print("Entered")

    #Callbacks for video feeds:
    def rightECMCallback(self,data):
        self.ecm_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.showFramesCallback()

    def leftECMCallback(self,data):
        self.ecm_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')        

    def leftExternalCallback(self,data):
        self.external_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def rightExternalCallback(self,data):
        self.external_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    #Superimpose the external view on bottom right corner of each ecm frame
    def superimposeView(self,ecm_frame,external_frame,left_right):
        #Input: ecm frame (ecm_frame); external camera frame: external_frame
        #Output: ecm frame with external camera frame on bottom right below ecm (superimposed_frame)
        if external_frame is not None and self.is_external_on:
            ecm_frame_new=cv2.resize(ecm_frame,(ECM_WIDTH,ECM_HEIGHT),interpolation=cv2.INTER_LINEAR)
            external_frame_new=cv2.resize(external_frame,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)
            # h,w=external_frame_new.shape[:2]
            # print("h: "+str(h)+" w: "+str(w))
            if self.is_undistort_available and self.is_undistort_on:               
                external_frame_new=self.undistortion_obj.undistortPickupCameraFrames(external_frame_new,left_right)
                # h,w=external_frame_new.shape[:2]
                # print("h: "+str(h)+" w: "+str(w))
                #external_frame_new=cv2.resize(external_frame_new,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR) #This line slows down the code a lot
            external_frame_new=self.unsharp_mask(external_frame_new,kernel_size=(self.sharpness_kernel_value,self.sharpness_kernel_value),\
                                                  sigma=self.sharpness_sigma_value,amount=3,threshold=0)
            
            superimposed_frame=np.zeros((VIEW_WINDOW_HEIGHT,VIEW_WINDOW_WIDTH,3),np.uint8)
            superimposed_frame[0:ECM_HEIGHT,int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2)):int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2))+ECM_WIDTH]=ecm_frame_new

            if left_right=='left': #We add a slight offset for the left/right to account for disparity
                superimposed_frame[-EXTERNAL_HEIGHT:,int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)-self.external_separation_val):int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)-self.external_separation_val)+EXTERNAL_WIDTH]=external_frame_new
            else: #right frame
                superimposed_frame[-EXTERNAL_HEIGHT:,int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)+self.external_separation_val):int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)+self.external_separation_val)+EXTERNAL_WIDTH]=external_frame_new

            
            #superimposed_frame[ECM_HEIGHT+1:,-EXTERNAL_WIDTH-((ECM_WIDTH-EXTERNAL_WIDTH)/2):]=external_frame

            # h_external=h*SIZE_REDUCTION_PERCENTAGE
            # w_external=w*SIZE_REDUCTION_PERCENTAGE
            # h_external=int(round(h_external))
            # w_external=int(round(w_external))
            # external_frame_new=cv2.resize(external_frame,(w_external,h_external),interpolation=cv2.INTER_LINEAR)
            # superimposed_frame=ecm_frame
            # superimposed_frame[-h_external:,-w_external:,:]=external_frame_new
        else:
            ecm_frame_new=cv2.resize(ecm_frame,(ECM_WIDTH,ECM_HEIGHT),interpolation=cv2.INTER_LINEAR)
            superimposed_frame=np.zeros((VIEW_WINDOW_HEIGHT,VIEW_WINDOW_WIDTH,3),np.uint8)
            superimposed_frame[0:ECM_HEIGHT,int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2)):int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2))+ECM_WIDTH]=ecm_frame_new

        return superimposed_frame
       
        # if external_frame is not None:
        #     h,w,_=external_frame.shape  #height/width of external camera frame

        #     h_external=h*SIZE_REDUCTION_PERCENTAGE
        #     w_external=w*SIZE_REDUCTION_PERCENTAGE
        #     h_external=int(round(h_external))
        #     w_external=int(round(w_external))
        #     external_frame_new=cv2.resize(external_frame,(w_external,h_external),interpolation=cv2.INTER_LINEAR)
        #     superimposed_frame=ecm_frame
        #     superimposed_frame[-h_external:,-w_external:,:]=external_frame_new
        # else:
        #     superimposed_frame=ecm_frame

        # return superimposed_frame   
    
    def unsharp_mask(self,image,kernel_size=(5,5),sigma=1.0,amount=1.0,threshold=0):
        blurred=cv2.GaussianBlur(image,kernel_size,sigma)

        sharpened=float(amount+1)*image-float(amount)*blurred
        sharpened=np.maximum(sharpened,ZEROS_ARRAY)
        sharpened=np.minimum(sharpened,MAX_ARRAY)
        #sharpened=sharpened.round().astype(np.uint8)
        if threshold>0:
            low_contrast_mask=np.absolute(image-blurred)<threshold
            np.copyto(sharpened,image,where=low_contrast_mask)
        
        return sharpened




if __name__=='__main__':
    rospy.init_node('ECM_VideoStreamer',anonymous=True)
    #rospy.Rate(10000)
    video_streamer=ECMVideoStreamer()
    
