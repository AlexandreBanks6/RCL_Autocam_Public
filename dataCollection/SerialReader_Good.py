import csv
import serial
import time
import sys

#import keyboard #Use Keyboard Library for windows
from pynput import keyboard #Use pynput library for linux

from datetime import datetime
import numpy as np
import dvrk

import DataLogger

#Flag to track if 'a' and 's' keys are pressed, uncomment for linux
q_pressed=False
s_pressed=False


###############Helper Methods###############
#Method to init arms
def setting_arms_state(arm):
    if arm.operating_state() == "DISABLED":
        arm.enable()
        arm.home()

#Uncomment these methods if running on linux, need to detect keyboard presses

def on_press(key):
    global q_pressed
    global s_pressed
    try:
        if key.char=='q':  #If 'a' key is pressed, set boolean to true
            q_pressed=True
            sys.stdout.flush()
            
        if key.char=='s': #If 's' key is pressed, set boolean to true
            s_pressed=True
            sys.stdout.flush()
    except AttributeError:
        pass


def on_release(key):
    global q_pressed
    global s_pressed
    try:
        if key.char=='q':  #If 'a' key is pressed, set boolean to true
            q_pressed=False
            sys.stdout.flush()
            #print("\r",end="")
        if key.char=='s': #If 's' key is pressed, set boolean to true
            s_pressed=False
            sys.stdout.flush()
            #Pprint("\r",end="")
    except AttributeError:
        pass

def clean_input_string(input_string):
    if input_string.startswith("sq"):
        return input_string[2:]
    else:
        return input_string


#Init Vars
csv_name='None' #Name of the csv where we are saving data, initialize to none
task_started=False #Toggles whether task is running or not
is_serial=True
is_csv=False

#Inits the datalogger object
dataLogger_obj=DataLogger.DataLogger()

#Inits the key press object, uncomment for linux
listener=keyboard.Listener(on_press=on_press,on_release=on_release)



#init arms
psm1 = dvrk.psm("PSM1")
psm3 = dvrk.psm("PSM3")
ecm = dvrk.ecm("ECM")

setting_arms_state(psm1)
setting_arms_state(psm3)
setting_arms_state(ecm)


#Connect to the serial port
#Change serial port if needed
ser_port=serial.Serial('/dev/ttyACM0',57600) #Use 'COM5' or similar if windows

#Starts the keyboard listener, uncomment for linux
listener.start()

#Read serial lines continually


while True:


    #Uncomment if running on windows    
    # if keyboard.is_pressed('s') and is_csv:
    #     task_started=not task_started         
    #     ser_port.write(b'\n') #Starts the task by sending an enter character
    #     is_serial=True
    #     time.sleep(0.6) #Allows release of enter

    # if keyboard.is_pressed('q') and is_csv:
    #     task_started=not task_started         
    #     ser_port.write(b'\n') #Stops the task by sending an enter character
    #     is_serial=True
    #     time.sleep(0.6) #Allows release of enter

    #Uncomment if running on Linux

    if q_pressed and is_csv:
        task_started=False      
        ser_port.write(b'\n') #Starts the task by sending an enter character
        is_serial=True
        time.sleep(0.2) #Allows release of enter
        q_pressed=False

    if s_pressed and is_csv:
        task_started=True         
        ser_port.write(b'\n') #Starts the task by sending an enter character
        is_serial=True
        time.sleep(0.2) #Allows release of enter
        s_pressed=False

    if is_serial: 

        ser_bytes=ser_port.readline() #Reads data until new line (\n)
        # Convert received bytes to text format
        decoded_bytes = ser_bytes.decode("utf-8").strip()    
        #print("decoded bytes: "+decoded_bytes)
    
        #Check what the decoded bytes are
        if "Enter CSV name to save data to (include .csv):" in decoded_bytes:
            #Prompt user to input the csv name
            csv_name=input("Enter CSV name to save data to (include .csv): ")
            csv_name=clean_input_string(csv_name)
            #print(csv_name)
            ser_port.write(csv_name.encode('utf-8'))
            dataLogger_obj.startCSV(csv_name)
            is_csv=True
            print("CSV Filename: "+dataLogger_obj.record_filename)
            time.sleep(0.2)
            #print("Done CSV")
            #Initialize new csv file
        #Checks for enter on keyboard to start/stop task
    
        elif "************************" in decoded_bytes:
            print(decoded_bytes)

        #elif "CSV Filename:" in decoded_bytes:
         #   print(decoded_bytes)

        #elif csv_name in decoded_bytes:
         #   print(dataLogger_obj.record_filename)
        
        elif "Press Enter to Start Task:" in decoded_bytes:
            print("Press 's' to Start Task:")   
            is_serial=False

        elif "Task Started" in decoded_bytes:
            print(decoded_bytes)

        elif "Press Enter to Stop Task:" in decoded_bytes:
            print("Press 'q' to Stop Task:")

        elif "Task Stopped" in decoded_bytes:
            is_csv=False
            print(decoded_bytes)

        elif "Started Serial Monitor with 57600 baud" in decoded_bytes:
            print(decoded_bytes)

        elif task_started:
            #Writing Data to .csv

            #Gets system time
            curr_time=datetime.now()
            curr_time=curr_time.strftime("%H:%M:%S.%f")

            #Decodes arduino list
            arduino_list=decoded_bytes.split(",")
            arduino_list=[int(item) for item in arduino_list]
            if len(arduino_list)>=5:
                #Data is in milliseconds, so we convert to seconds
                arduino_list[1]=arduino_list[1]/1000
                arduino_list[4]=arduino_list[4]/1000

                #Gets frames and joint variables
                try:
                    base_T_ecm=ecm.measured_cp()
                except Exception as e:
                    print("Unable to read ecm: "+str(e))
                    base_T_ecm=["NaN"]*12
            
                try:
                    ecm_T_psm1=psm1.measured_cp()
                except Exception as e:
                    print("Unable to read psm1: "+str(e))
                    ecm_T_psm1=["NaN"]*12

                try:
                    ecm_T_psm3=psm3.measured_cp()
                except Exception as e:
                    print("Unable to read psm3: "+str(e))
                    ecm_T_psm3=["NaN"]*12

                #Joint vars
                try:
                    joint_vars_psm1=psm1.measured_js()[0]
                    jaw_angle_psm1=psm1.jaw.measured_js()[0]
                    psm1_joints=np.concatenate((joint_vars_psm1,jaw_angle_psm1))
                    psm1_joints=psm1_joints.tolist()
                except Exception as e:
                    print("Unable to read psm1 joints: "+str(e))
                    psm1_joints=["NaN"]*7

                try:
                    joint_vars_psm3=psm3.measured_js()[0]
                    jaw_angle_psm3=psm3.jaw.measured_js()[0]
                    psm3_joints=np.concatenate((joint_vars_psm3,jaw_angle_psm3))
                    psm3_joints=psm3_joints.tolist()
                except Exception as e:
                    print("Unable to read psm3 joints: "+str(e))
                    psm3_joints=["NaN"]*7

                try:
                    ecm_joints=ecm.measured_js()[0]
                    ecm_joints=ecm_joints.tolist()
                except Exception as e:
                    print("Unable to read ecm joints: "+str(e))
                    ecm_joints=["NaN"]*4



                dataLogger_obj.writeRow(arduino_list,curr_time,base_T_ecm,ecm_T_psm1,ecm_T_psm3,psm1_joints,psm3_joints,ecm_joints)





