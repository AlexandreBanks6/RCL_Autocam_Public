import csv
import serial
import time

csv_filepath='CollisionData\\Pilot01\\' #Change to correct subfolder
csv_filename='None'
csv_name='None' #Name of the csv where we are saving data, initialize to none
CSV_HEADER=["Sample #","Task Duration (s)","Collision (binary)","Cumulative Collision Time (s)","System Time"] #Header of the csv file

task_started=False



#Connect to the serial port
ser_port=serial.Serial('COM5',57600) #COM must match what arduino IDE says


#Read lines continually

while True:
    ser_bytes=ser_port.readline() #Reads data until new line (\n)
    # Convert received bytes to text format
    decoded_bytes = ser_bytes.decode("utf-8").strip()
    #print(decoded_bytes)
    #Check what the decoded bytes are
    if "Enter CSV name to save data to (include .csv):" in decoded_bytes:
        #Prompt user to input the csv name
        csv_name=input("Enter CSV name to save data to (include .csv): ")
        ser_port.write(csv_name.encode('utf-8'))
        csv_filename=csv_filepath+csv_name
        with open(csv_filename,'w',newline='') as file_obj:
            writer_obj=csv.writer(file_obj)
            writer_obj.writerow(CSV_HEADER)
            writer_obj.writerow(["","","","",""])
            file_obj.close()
        #Initialize new csv file
    elif "************************" in decoded_bytes:
        print(decoded_bytes)
    elif "CSV Filename:" in decoded_bytes:
        print(decoded_bytes)
    elif csv_name in decoded_bytes:
        print(csv_filename)
    elif "Press Enter to Start Task:" in decoded_bytes:
        input("Press Enter to Start Task: ")
        time.sleep(0.03) #Allows release of enter
        ser_port.write(b'\n') #Starts the task by sending an enter character
        
    elif "Task Started" in decoded_bytes:
        print(decoded_bytes)
    elif "Press Enter to Stop Task:" in decoded_bytes:
        input("Press Enter to Stop Task: ")
        ser_port.write(b'\n') #Sends newline to stop task recording
        time.sleep(0.03) #allows release of enter    elif "Task Stopped" in decoded_bytes:
    elif "Task Stopped" in decoded_bytes:
        print(decoded_bytes)
    elif "Started Serial Monitor with 57600 baud" in decoded_bytes:
        print(decoded_bytes)
    else:
        #Writing Data to .csv
        #print(decoded_bytes)
        data_list=decoded_bytes.split(",")
        data_list=[int(item) for item in data_list]
        #print(data_list)
        #Data is in milliseconds, so we convert to seconds
        data_list[1]=data_list[1]/100
        data_list[3]=data_list[3]/100
        print(data_list)
        with open(csv_filename,'a',newline='') as file_obj:
            writer_obj=csv.writer(file_obj)
            writer_obj.writerow(data_list)
            file_obj.close()

        
    

