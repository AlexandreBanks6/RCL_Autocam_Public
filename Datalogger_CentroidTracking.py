import glm
import numpy as np
import cv2
from datetime import datetime
import csv
import os
import serial
import time
import sys

FRAME_FPS=30 #Randomly set the frame rate of the video
SIZE_REDUCTION_PERCENTAGE_ECM=0.365 #Amount that external frame is reduced from original
SIZE_REDUCTION_PERCENTAGE_EXTERNAL=0.4
EXTERNAL_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*1280))
EXTERNAL_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*960))


repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column
MOTION_HEADER=["PC Time","Left Frame #","Left Aruco ID","leftcam_T_aruco"]+repeat_string+\
["aruco_T_ring_left"]+repeat_string+["u_left","v_left"]+\
["Right Frame #","Right Aruco ID","rightcam_T_aruco"]+repeat_string+\
["aruco_T_ring_right"]+repeat_string+["u_right","v_right"]


class Datalogger_CentroidTracking:
    def __init__(self,mtx_left,dist_left,mtx_right,dist_right):
        print('Init Datalogger')
        self.record_filename=None
        self.left_external_filename=None
        self.right_external_filename=None
        self.left_video_writer=None
        self.right_video_writer=None
        self.file_count=1
        self.mtx_left=mtx_left
        self.dist_left=dist_left
        self.mtx_right=mtx_right
        self.dist_right=dist_right



    def initRecording(self,root_path):

        #file_count comes from ROS topic published by PC1

        if not os.path.exists(root_path):
            os.makedirs(root_path)

        #Initialize a new CSV file to save PC1 data to
        file_name=root_path+'/Data_'+str(self.file_count)+'.csv'
        #Gets a new filename
        while True:
            if os.path.isfile(file_name):
                self.file_count+=1
                file_name=root_path+'/Data_'+str(self.file_count)+'.csv'                
            else:
                break

        self.record_filename=file_name  #Creates a new data csv

        with open(self.record_filename,'w') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(MOTION_HEADER)

            writer_object.writerow([""]) #Blank Space

            #mtx_left
            mtx_left=self.mtx_left.flatten().tolist()
            mtx_left=self.convertListToStringList(mtx_left)
            writer_object.writerow(["mtx_left"]+mtx_left)

            #dist_left
            dist_left=self.dist_left.tolist()[0]
            dist_left=self.convertListToStringList(dist_left)
            writer_object.writerow(["dist_left"]+dist_left)

            #mtx_right
            mtx_right=self.mtx_right.flatten().tolist()
            mtx_right=self.convertListToStringList(mtx_right)
            writer_object.writerow(["mtx_right"]+mtx_right)

            #dist_right
            dist_right=self.dist_right.tolist()[0]
            dist_right=self.convertListToStringList(dist_right)
            writer_object.writerow(["dist_right"]+dist_right)

            writer_object.writerow([""]) #Blank Space



            file_object.close()

        #Sets the filename (and path) for the left/right ECM video
        self.left_external_filename=root_path+'/LeftExternal_'+str(self.file_count)+'.avi'
        self.right_external_filename=root_path+'/RightExternal_'+str(self.file_count)+'.avi'
        #initializes the video writer objects
        fourcc=cv2.VideoWriter_fourcc(*'MJPG')
        self.left_video_writer=cv2.VideoWriter(self.left_external_filename,fourcc,FRAME_FPS,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT))
        self.right_video_writer=cv2.VideoWriter(self.right_external_filename,fourcc,FRAME_FPS,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT))


    def writeRow(self,pc_time,left_frame_number,left_id,leftcam_T_aruco,aruco_T_ring_left,u_left,v_left,right_frame_number,right_id,rightcam_T_aruco,aruco_T_ring_right,u_right,v_right):

        if leftcam_T_aruco is not None:
            leftcam_T_aruco=self.convertHomogeneousToCSVROW(leftcam_T_aruco)
        else:
            leftcam_T_aruco=["NaN"]*12
        if aruco_T_ring_left is not None:
            aruco_T_ring_left=self.convertHomogeneousToCSVROW(aruco_T_ring_left)
        else:
            aruco_T_ring_left=["NaN"]*12

        if rightcam_T_aruco is not None:
            rightcam_T_aruco=self.convertHomogeneousToCSVROW(rightcam_T_aruco)
        else:
            rightcam_T_aruco=["NaN"]*12

        if aruco_T_ring_right is not None:
            aruco_T_ring_right=self.convertHomogeneousToCSVROW(aruco_T_ring_right)
        else:
            aruco_T_ring_right=["NaN"]*12



        row_to_write=[str(pc_time),str(left_frame_number),str(left_id),""]+leftcam_T_aruco+[""]+aruco_T_ring_left+[str(u_left),str(v_left)]+[str(right_frame_number),str(right_id),""]+rightcam_T_aruco+[""]+aruco_T_ring_right+[str(u_right),str(v_right)]
        row_to_write=list(row_to_write)

        with open(self.record_filename,'a') as file_obj:
            writer_object=csv.writer(file_obj)
            writer_object.writerow(row_to_write)
            file_obj.close()


    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        return string_list
        
    def convertListToStringList(self,list):
        string_list=[str(val) for val in list]
        return string_list
    