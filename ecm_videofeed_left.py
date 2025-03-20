'''
Title: ECM + External Camera Video Streamer
Date: July 4th, 2024
Authors: Alexandre Banks & Randy Moore
Description: Takes in the left/right ECM from the dVRK and the left/right video feed 
from an external camera and superimposes the external camera below the ECM frame

Steps to Setup ROS for Cameras:

1. $roscore on dVRK-pc2
2. $roslaunch dvrk_robot jhu_daVinci_video.launch rig_name:=ubc_dVRK_ECM
    - Gets the left/right ECM feeds with topics:
    'ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
    'ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
3. ssh pi@192.168.0.10 => Enters left pi
 3.2 export ROS_IP=192.168.0.10
 3.3 export ROS_MASTER_URI=http://192.168.0.11:11311
 3.4 roslaunch raspicam_node camerav2_1280x960.launch
    - Starts left external camera video feed with ros topic:
    'raspicam_node_left/image/compressed'

4. ssh pi@192.168.0.12 => Enters right pi
 4.2 export ROS_IP=192.168.0.12
 4.3 export ROS_MASTER_URI=http://192.168.0.11:11311
 4.4 roslaunch raspicam_node camerav2_1280x960.launch
    - Starts right external camera video feed with ros topic:
    'raspicam_node_right/image/compressed'

Note: Password for both pi's is mech464
'''


import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

#ROS Topics
RightECM_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftECM_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
RightExternal_Topic='raspicam_node_right/image/compressed'
LeftExternal_Topic='raspicam_node_left/image/compressed'
SIZE_REDUCTION_PERCENTAGE=0.5 #Amount that external frame is reduced from original

class ECMVideoStreamer:
    def __init__(self):


        self.bridge=CvBridge()


        #Subscribing to ROS Topics
        rospy.Subscriber(name=LeftECM_Topic,data_class=CompressedImage,callback=self.leftECMCallback,queue_size=1,buff_size=2**20)
      
        
        rospy.Subscriber(name=LeftExternal_Topic,data_class=CompressedImage,callback=self.leftExternalCallback,queue_size=1,buff_size=2**20)

        #Frames Init
        self.external_frame_left = None
        self.ecm_frame_left = None 

        print("Ros started")

        rospy.spin()
        


    def showFramesCallback(self):
        
        if self.ecm_frame_left is not None:
            superimposed_left=self.superimposeView(self.ecm_frame_left,self.external_frame_left)
            #Showing Frames
            cv2.imshow("left_eye",superimposed_left)
            keys=cv2.waitKey(1) & 0xFF
            if keys==ord('q'):
                print("Entered")



    def leftECMCallback(self,data):
        self.ecm_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')        
        self.showFramesCallback()

    def leftExternalCallback(self,data):
        self.external_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')


    #Superimpose the external view on bottom right corner of each ecm frame
    def superimposeView(seld,ecm_frame,external_frame):
        #Input: ecm frame (ecm_frame); external camera frame: external_frame
        #Output: ecm frame with external camera frame on bottom right corner (superimposed_frame)
        if external_frame is not None:
            h,w,_=external_frame.shape  #height/width of external camera frame

            h_external=h*SIZE_REDUCTION_PERCENTAGE
            w_external=w*SIZE_REDUCTION_PERCENTAGE
            h_external=int(round(h_external))
            w_external=int(round(w_external))
            external_frame_new=cv2.resize(external_frame,(w_external,h_external),interpolation=cv2.INTER_LINEAR)
            superimposed_frame=ecm_frame
            superimposed_frame[-h_external:,-w_external:,:]=external_frame_new
        else:
            superimposed_frame=ecm_frame

        return superimposed_frame   



if __name__=='__main__':
    rospy.init_node('ECM_VideoStreamer',anonymous=True)
    #rospy.Rate(10000)
    print("Started")
    video_streamer=ECMVideoStreamer()
    
