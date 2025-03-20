import csv
import os
from datetime import datetime
import numpy as np
import pandas as pd

repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"]
CSV_HEADER=["Elapsed Time","System Time","Optimizer Flag","Solver Success","No Go Flag","Below Floor Flag","Orientation Flag","Proximity Flag","ecm_T_psm1_raw"]+\
    repeat_string+["ecm_T_psm1_filtered"]+repeat_string+["ecm_T_psm3_raw"]+repeat_string+["ecm_T_psm_rcm"]+repeat_string+["ecm_T_psm3_des"]+repeat_string+["ecm_T_psm3_des_proximity"]+repeat_string+\
        ["ecm_T_psm3_command"]+repeat_string+["ecm_T_ring_target"]+repeat_string+["ecm_T_world"]+repeat_string+["q_curr",'q0','q1','q2','q3','q4','q5']+\
            ['q_solverOutput','q0','q1','q2','q3','q4','q5']

NOT_A_TRANSFORM=np.array([[-1,-1,-1,-1]],
[-1,-1,-1,-1],
[-1,-1,-1,-1],
[-1,-1,-1,-1])

ROOT_PATH='optimizationData'

class OptimizationDataLogger:
    def __init__(self):
        print("Init Optimization Datalogger")
        self.record_filename=None
        self.file_count=1
        self.read_row_count=0

    def initRecording(self,calib_offset,psm3_T_cam,no_go_points,psm1_T_R):
        #Check if path exists
        if not os.path.exists(ROOT_PATH):
            os.makedirs(ROOT_PATH)
        #Initialize a new CSV file to save PC1 data to
        file_name=ROOT_PATH+'/Data_'+str(self.file_count)+'.csv'
        #Gets a new filename
        while True:
            if os.path.isfile(file_name):
                self.file_count+=1
                file_name=ROOT_PATH+'/Data_'+str(self.file_count)+'.csv'             
            else:
                break
        self.record_filename=file_name

        #Inits Header
        with open(self.record_filename,'w',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(CSV_HEADER)

             writer_object.writerow([""]) #Blank Space

            ########Writes the calibration params at top of .csv#######

            #Offset between psm1 and psm3
            writer_object.writerow(["offset:"]+[str(offset[0]),str(offset[1]),str(offset[2])])
            writer_object.writerow(["no go points:"]+[str(no_go_points[0]),str(no_go_points[1]),str(no_go_points[2]),str(no_go_points[3]),str(no_go_points[4])])
            writer_object.writerow(["psm3_T_cam:"]+self.convertHomogeneousToStringCSVROW(psm3_T_cam))
            writer_object.writerow(["psm1_T_R:"]+self.convertHomogeneousToStringCSVROW(psm1_T_R))


            writer_object.writerow([""]) #Blank Space
            file_object.close()


    def writeRow(self,data_list):
        

        '''
        data_list contains a list where each index is:, in a list format

        elapsed_time,
        [optimizer_flag,
        solver_success,
        no_go_flag,
        below_floor_flag,
        orientation_flag,
        proximity_flag],
        ecm_T_psm1_raw,
        ecm_T_psm1_filtered,
        ecm_T_psm3_raw,
        ecm_T_psm_rcm,
        ecm_T_psm3_des,
        ecm_T_psm3_des_proximity,
        ecm_T_psm3_command,
        ecm_T_ring_target,
        ecm_T_world,
        q_curr,
        q_solveroutput
        '''
        #Gets system time
        curr_time=datetime.now()
        curr_time=curr_time.strftime("%H:%M:%S.%f")

        row_to_write=[] #Inits the row that we will write data to

        #Writes elapsed time
        row_to_write.append(data_list[0])
        #Writes current time
        row_to_write.append(curr_time)
        #Writes flags
        row_to_write.extend(data_list[1])
        row_to_write.append("")

        #ecm_T_psm1_raw
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[2]))
        row_to_write.append("")
        #ecm_T_psm1_filtered
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[3]))
        row_to_write.append("")

        #ecm_T_psm3_raw
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[4]))
        row_to_write.append("")

        #ecm_T_psm_rcm
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[5]))
        row_to_write.append("")

        #ecm_T_psm3_des
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[6]))
        row_to_write.append("")

        #ecm_T_psm3_des_proximity
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[7]))
        row_to_write.append("")

        #ecm_T_psm3_command
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[8]))
        row_to_write.append("")

        #ecm_T_ring_target
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[9]))
        row_to_write.append("")

        #ecm_T_world
        row_to_write.extend(self.convertHomogeneousToCSVROW(data_list[10]))
        row_to_write.append("")

        #q_curr
        row_to_write.extend(data_list[11])
        row_to_write.append("")

        #q_solveroutput
        row_to_write.extend(data_list[12])
        row_to_write.append("")       
        
        with open(self.record_filename,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(row_to_write)
            file_object.close()

    def initReading(self,filname=None):
        #Filename is pre-defined filename, if None use highest numbered filename
        if filname is not None:
            self.record_filename=ROOT_PATH+'/'+filname+'.csv'
        else:
            file_count=1
            file_name=ROOT_PATH+'/Data_'+str(file_count)+'.csv'
            while True:
                if os.path.isfile(file_name):
                    file_count+=1
                    file_name=ROOT_PATH+'/Data_'+str(file_count)+'.csv'
                else:
                    break
            self.record_filename=ROOT_PATH+'/Data_'+str(file_count-1)+'.csv'
        #Initializes the counter to index the rows:
        self.read_row_count=0
        # self.total_rows = sum(1 for row in open(self.record_filename)) - 1
        
        self.csv_obj=pd.read_csv(self.record_filename)
        self.ik_indices=self.csv_obj.index[self.csv_obj['IK Trigered']==True].to_list() #Used to find which indices are only when IK is triggered
        self.total_rows=self.csv_obj.shape[0]

    def readDataRow(self,row_index=None):
        if row_index is None:
            self.read_row_count+=1
            row_index=self.read_row_count

        if row_index>=self.total_rows:
            #Reached end of the row
            success=False
            system_time=None
            q_des=None
            T_des=None
            T_target=None
            worldFrame=None
            ECM_T_PSM_RCM=None
            psm3_T_cam=None
            offset=None
            IK_Triggered=None
            ECM_T_PSM3=None
        else:
            success=True
            #data_row=pd.read_csv(self.record_filename,skiprows=row_index)
            #data_list=data_row.iloc[0].to_list()
            data_list=self.csv_obj.iloc[row_index].to_list()

            #Returns the data as a list such that:
            '''
            q_des, 
            T_des, 
            T_target, 
            worldFrame,
            ECM_T_PSM_RCM, 
            psm3_T_cam,
            offset
            '''
            system_time=data_list[0]
            q_des=data_list[2:8]
            T_des=data_list[9:21]
            T_target=data_list[22:34]
            worldFrame=data_list[35:47]
            ECM_T_PSM_RCM=data_list[48:60]
            psm3_T_cam=data_list[61:73]
            offset=data_list[74:77]
            IK_Triggered=data_list[77]
            ECM_T_PSM3=data_list[79:91]


            q_des=np.array(q_des,dtype=np.float64)
            T_des=self.ConvertDataRow_ToNPFrame(T_des)
            T_target=self.ConvertDataRow_ToNPFrame(T_target)
            worldFrame=self.ConvertDataRow_ToNPFrame(worldFrame)
            ECM_T_PSM_RCM=self.ConvertDataRow_ToNPFrame(ECM_T_PSM_RCM)
            psm3_T_cam=self.ConvertDataRow_ToNPFrame(psm3_T_cam)
            offset=np.array(offset,dtype=np.float64)   
            ECM_T_PSM3=self.ConvertDataRow_ToNPFrame(ECM_T_PSM3)

        # print("q_des: "+str(q_des))
        # print("T_des: "+str(T_des))
        # print("T_target: "+str(T_target))
        # print("worldFrame: "+str(worldFrame))
        # print("ECM_T_PSM_RCM: "+str(ECM_T_PSM_RCM))
        # print("psm3_T_cam: "+str(psm3_T_cam))
        # print("offset: "+str(offset))
        # print("ECM_T_PSM3: "+str(ECM_T_PSM3))
        # print("IK Triggered: "+str(IK_Triggered))

        return success,system_time,q_des,T_des,T_target,worldFrame,ECM_T_PSM_RCM,psm3_T_cam,offset,IK_Triggered,ECM_T_PSM3
        




    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        t_list=[transform[0,3],transform[1,3],transform[2,3],\
                    transform[0,0],transform[0,1],transform[0,2],\
                    transform[1,0],transform[1,1],transform[1,2],\
                    transform[2,0],transform[2,1],transform[2,2]]
        
        
        return t_list

    def convertHomogeneousToStringCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        
        
        return string_list
    

    def ConvertDataRow_ToNPFrame(self,data_list):
        transform=np.array([[data_list[3],data_list[4],data_list[5],data_list[0]],
                           [data_list[6],data_list[7],data_list[8],data_list[1]],
                           [data_list[9],data_list[10],data_list[11],data_list[2]],
                           [0,0,0,1]],dtype=np.float64)

        return transform
        

    

    
