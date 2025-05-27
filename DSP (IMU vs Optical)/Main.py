from collections import deque
import os
import time
from socket import *
from Classes.BODYPART import BODYPART
from Utils.Calibration import check_and_calibrate
from Utils.Calibration import load_pose_calibration
from Classes.Human import Human
import Utils.Esp32 as Esp32
import Utils.Cam as Cam
import Utils.Button as button
import threading
import numpy as np
import pandas as pd
import csv
import keyboard


name_map = {
    'mixamorig:LeftArm': 'l_shoulder',
    'mixamorig:LeftForeArm': 'l_elbow',
    'mixamorig:RightArm': 'r_shoulder',
    'mixamorig:RightForeArm': 'r_elbow',
}

os.system("cls")
data = ""
rawData = deque(maxlen=1000)
calcData = deque(maxlen=1000)
ahmad = Human('IMUQ')
mustafa = Human('CUSTOM')


def get_data(bone_rotations):
    converted = {}
    for line in bone_rotations.strip().splitlines():
        # split into ["mixamorig", "LeftArm", "39,0,0"]
        parts = line.split(":", 2)
        if len(parts) != 3:
            continue

        bone_key = parts[0] + ":" + parts[1]
        value_str = parts[2]
        target_name = name_map.get(bone_key)
        if target_name:
            # parse "39,0,0" → (39.0, 0.0, 0.0)
            converted[target_name] = tuple(map(float, value_str.split(",")))

    return converted


def format_bone_data(bone_name, obj: BODYPART):

    # if bone_name == "mixamorig:Head":
    #     return f"{bone_name}:{getattr(obj, 'yaw', 0)},-{getattr(obj, 'pitch', 0)},{getattr(obj, 'roll', 0)}"
    p: int = obj.pitch
    r: int = obj.roll
    y: int = obj.yaw
    return f"{bone_name}:{p},{r},{y}"


def print_loop(ahmad: Human, mustafa: Human):
    global data
    global calcData
    while True:
        os.system("cls")
    
        
        # ESP32 data
        try:
            print("\n________________________Sensor Raw Data")
            # Esp32.printData(data, {"chest", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow"})
            print(data)
        except:
            print("No ESP32 data")



        # Human data
        # ahmad.__print_data__()
        print("\n________________________Sensor YPR")
        # mustafa.__print_data__()
        # print(calcData[-1])

        
        # Camera data
        if Cam.CamYPR:
            print("\n________________________Camera YPR")
            # Cam.print_data()
            print(Cam.get_data())

        if recording:
            print("\n!!!!! RECORDING !!!!!")

        
        time.sleep(0.2)
       


def run_tracker():
    Cam.run_pose_tracking()
    


# main loop ______________________________________________________________________________
def sensor_mopcap():
    global data
    global rawData
    global ahmad
    global mustafa
    

    Esp32.register_esp()
    # check_and_calibrate(mustafa)
    load_pose_calibration(mustafa, "files/Tpose.yml")
    load_pose_calibration(mustafa, "files/Ipose.yml")
    load_pose_calibration(mustafa, "files/Lpose.yml")

    load_pose_calibration(ahmad, "files/Tpose.yml")
    load_pose_calibration(ahmad, "files/Ipose.yml")
    load_pose_calibration(ahmad, "files/Lpose.yml")


    # Inertial MoCap _______________________________________________________
    inertial_serverIP = '127.0.0.1' 
    inertial_serverPort = 25000
    inertial_socket = socket(AF_INET, SOCK_DGRAM)



    prev_time = time.time()
    frame_count = 0
    last_frame_count = 0


    threading.Thread(target=run_tracker).start() #Cam thread
    threading.Thread(target=print_loop, args=(ahmad, mustafa), daemon=True).start() #Data Printing Thread

    while True:
        start_time = time.time() 
        
        # Get ESP32 data
        try:
            data = Esp32.read_esp32()
            # data = Esp32.get_random_data()
            rawData.append(data)
            ahmad.__update_data__(data, False)
            mustafa.__update_data__(data, False)
        except Exception as e:
            print(f"ESP32 Error: {str(e)}")
        

        
        # Data transfer ________________________________________________________________
        try:
            # Inertial data
            transfer_data_list = [
                format_bone_data("mixamorig:Spine", mustafa.chest),
                format_bone_data("mixamorig:LeftForeArm", mustafa.l_elbow),
                format_bone_data("mixamorig:LeftArm", mustafa.l_shoulder),
                format_bone_data("mixamorig:RightForeArm", mustafa.r_elbow),
                format_bone_data("mixamorig:RightArm", mustafa.r_shoulder)
            ]
            final_transfer_data = "\n".join(transfer_data_list)
            inertial_socket.sendto(final_transfer_data.encode('utf-8'), 
                                (inertial_serverIP, inertial_serverPort))
            calcData.append(get_data(final_transfer_data))
        except Exception as e:
            print(f"Network Error: {str(e)}")
        ##### data transfer __________________________________________________________________


        # Calculate FPS ______________________________________________________________________
        frame_count += 1
        elapsed_time = time.time() - prev_time

        if elapsed_time >= 1.0:  # Update FPS every second
            print(f"\nFPS: {frame_count}")
            frame_count = 1
            prev_time = time.time()




#___________________________________________________table____

columns = ['timestamp', 'l_shoulder', 'r_shoulder', 'l_elbow', 'r_elbow']
columnsFull = ['timestamp', 'l_shoulder', 'r_shoulder', 'l_elbow', 'r_elbow', 'chest']
BATCH_SIZE = 100

CamYPR = pd.DataFrame(columns=columns)
ImuYPR = pd.DataFrame(columns=columnsFull)
ImuRAW = pd.DataFrame(columns=columnsFull)

imu_raw_list = []
imu_ypr_list = []
cam_ypr_list = []

# global flag
recording = False


#____________________________________________________________________________
def flush_to_csv():
    """Write accumulated data to CSV files."""
    global imu_raw_list, imu_ypr_list, cam_ypr_list
    
    def write_csv(filename, data_list, fieldnames):
        if not data_list:
            return
        file_exists = os.path.isfile(filename)
        with open(filename, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerows(data_list)
    
    # Write datasets with correct columns
    write_csv('dataset/ImuRAW.csv', imu_raw_list, columnsFull)
    write_csv('dataset/ImuYPR.csv', imu_ypr_list, columnsFull)
    write_csv('dataset/CamYPR.csv', cam_ypr_list, columns)
    
    # Clear the lists
    imu_raw_list.clear()
    imu_ypr_list.clear()
    cam_ypr_list.clear()




#____________________________________________________________________________
if __name__ == "__main__":

    # sensor_mopcap()
    threading.Thread(target=sensor_mopcap).start() #Suit thread
    

    time.sleep(20)
    recording = True
    while True:
        if recording:
            timestamp = time.time()

            # Flatten RAW IMU data for saving
            imu_raw_flat = {'timestamp': timestamp}
            for part in ['l_shoulder', 'r_shoulder', 'l_elbow', 'r_elbow', 'chest']:
                imu_raw_flat[part] = ','.join(f'{v:.2f}' for v in rawData[-1].get(part, []))

            # YPR data (already 3-tuple)
            imu_ypr_flat = {'timestamp': timestamp}
            for part in ['l_shoulder', 'r_shoulder', 'l_elbow', 'r_elbow', 'chest']:
                imu_ypr_flat[part] = ','.join(str(v) for v in calcData[-1].get(part, (0, 0, 0)))

            # Camera YPR
            cam_ypr_flat = {'timestamp': timestamp}
            for part in ['l_shoulder', 'r_shoulder', 'l_elbow', 'r_elbow']:
                cam_ypr_flat[part] = ','.join(str(v) for v in Cam.get_data().get(part, (0, 0, 0))) 

            # Append to batch
            imu_raw_list.append(imu_raw_flat)
            imu_ypr_list.append(imu_ypr_flat)
            cam_ypr_list.append(cam_ypr_flat)

            time.sleep(0.05)

            # Flush to CSV if any list reaches batch size
            if len(imu_raw_list) >= BATCH_SIZE or len(imu_ypr_list) >= BATCH_SIZE or len(cam_ypr_list) >= BATCH_SIZE:
                flush_to_csv()






        