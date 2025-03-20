import csv
import os
import tf_conversions.posemath as pm
#Class to log all data collected in the autocam study
#stores the errors and 3 transforms
repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column

#Header of the csv file
CSV_HEADER=["Sample #","Task Duration (s)","Collisions (binary)","Collisions (increments)","Cumulative Collision Time (s)","System Time","base_T_ecm"]+\
repeat_string + ["ecm_T_psm1"]+repeat_string+["ecm_T_psm3"]+repeat_string+\
["psm1_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+["psm3_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+\
["ecm_joints"]+["q1","q2","q3","q4"]

csv_file_root='dataCollection/ParticipantData/' #May need to change directory if you are running in the "dataCollection" folder

class DataLogger:
    def __init__(self):
        self.directory_name=csv_file_root
        self.record_filename=None

        #Gets user input on the subfolder name
        subfolder_name=input("Enter the subfolder name to store participant's data (e.g. Pilot01): ")
        subfolder_name=str(subfolder_name)
        self.directory_name=self.directory_name+subfolder_name
        #Checks if the path exists, if not create it 
        if not os.path.exists(self.directory_name):
             os.mkdir(self.directory_name)

    def startCSV(self,file_name):
        #inits the csv file
        self.record_filename=self.directory_name+'/'+file_name
        with open(self.record_filename,'w',newline='') as file_obj:
                writer_obj=csv.writer(file_obj)
                writer_obj.writerow(CSV_HEADER)
                file_obj.close()

    def writeRow(self,arduino_list,system_time,base_T_ecm,ecm_T_psm1,ecm_T_psm3,psm1_joints,psm3_joints,ecm_joints):
        #Joints are already passed as list
        
        #Starts row that we will write
        row_to_write=arduino_list
        row_to_write.append(system_time)

        #Converts PyKDL frames to numpy and then format for writing to csv
        if isinstance(base_T_ecm,list):
             base_T_ecm_list=base_T_ecm
        else:
            base_T_ecm=pm.toMatrix(base_T_ecm)
            base_T_ecm_list=self.convertHomogeneousToCSVROW(base_T_ecm)

        if isinstance(ecm_T_psm1,list):
             ecm_T_psm1_list=ecm_T_psm1
        else:
            ecm_T_psm1=pm.toMatrix(ecm_T_psm1)
            ecm_T_psm1_list=self.convertHomogeneousToCSVROW(ecm_T_psm1)

        if isinstance(ecm_T_psm3,list):
            ecm_T_psm3_list=ecm_T_psm3
        else:
            ecm_T_psm3=pm.toMatrix(ecm_T_psm3)
            ecm_T_psm3_list=self.convertHomogeneousToCSVROW(ecm_T_psm3)



        #Appending data to the list to write
        row_to_write.append("")
        row_to_write.extend(base_T_ecm_list)
        row_to_write.append("")
        row_to_write.extend(ecm_T_psm1_list)
        row_to_write.append("")
        row_to_write.extend(ecm_T_psm3_list)
        row_to_write.append("")
        row_to_write.extend(psm1_joints)
        row_to_write.append("")
        row_to_write.extend(psm3_joints)
        row_to_write.append("")
        row_to_write.extend(ecm_joints)

        with open(self.record_filename,'a',newline='') as file_obj:
            writer_obj=csv.writer(file_obj)
            writer_obj.writerow(row_to_write)
            file_obj.close()

    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[transform[0,3],transform[1,3],transform[2,3],\
                    transform[0,0],transform[0,1],transform[0,2],\
                    transform[1,0],transform[1,1],transform[1,2],\
                    transform[2,0],transform[2,1],transform[2,2]]
        
        
        return string_list


    