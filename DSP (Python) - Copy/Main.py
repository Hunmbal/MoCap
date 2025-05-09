import os
import time
from socket import *
from Classes.BODYPART import BODYPART
from Utils.Calibration import check_and_calibrate
from Utils.Calibration import load_pose_calibration
from Classes.Human import Human
import Utils.Esp32 as Esp32
import threading
import numpy as np


os.system("cls")


def format_bone_data(bone_name, obj: BODYPART):

    # if bone_name == "mixamorig:Head":
    #     return f"{bone_name}:{getattr(obj, 'yaw', 0)},-{getattr(obj, 'pitch', 0)},{getattr(obj, 'roll', 0)}"
    p: int = obj.pitch
    r: int = obj.roll
    y: int = obj.yaw


    return f"{bone_name}:{p},{r},{y}"





def print_loop(ahmad: Human, mustafa: Human):
    global frame_count
    time.sleep(0.2)  # Print every 0.5 seconds (adjust as you like)
    global data
    while True:
        os.system("cls")
        ahmad.__print_data__()
        print("\n")
        mustafa.__print_data__()
        print("\n")
        Esp32.printData(data, {"chest", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow"})

        time.sleep(0.2)  # Print every 0.5 seconds (adjust as you like)
       





if __name__ == "__main__":

    ahmad = Human('IMUQ')
    mustafa = Human('CUSTOM')

    Esp32.register_esp()
    check_and_calibrate(mustafa)
    load_pose_calibration(mustafa, "Tpose.yml")
    load_pose_calibration(mustafa, "Ipose.yml")
    load_pose_calibration(mustafa, "Lpose.yml")

    load_pose_calibration(ahmad, "Tpose.yml")
    load_pose_calibration(ahmad, "Ipose.yml")
    load_pose_calibration(ahmad, "Lpose.yml")

    serverIP = '127.0.0.1' 
    serverPort = 25000
    clientSocket = socket(AF_INET, SOCK_DGRAM)


    prev_time = time.time()
    frame_count = 0
    last_frame_count = 0

    threading.Thread(target=print_loop, args=(ahmad, mustafa), daemon=True).start()


    # main loop ______________________________________________________________________________
    while True:
        start_time = time.time() 
        data = Esp32.read_esp32()  # Simulated random data
        ahmad.__update_data__(data, False)
        mustafa.__update_data__(data, False)

        
    #   ##### data transfer __________________________________________________________________
        transfer_data_list = []
        transfer_data_list.append(format_bone_data("mixamorig:Spine", mustafa.chest))
        transfer_data_list.append(format_bone_data("mixamorig:LeftForeArm", mustafa.l_elbow))
        transfer_data_list.append(format_bone_data("mixamorig:LeftArm", mustafa.l_shoulder))
        transfer_data_list.append(format_bone_data("mixamorig:RightForeArm", mustafa.r_elbow))
        transfer_data_list.append(format_bone_data("mixamorig:RightArm", mustafa.r_shoulder))
        final_transfer_data = "\n".join(transfer_data_list)
        clientSocket.sendto(final_transfer_data.encode('utf-8'), (serverIP, serverPort))
        ##### data transfer __________________________________________________________________


        # Calculate FPS ______________________________________________________________________
        frame_count += 1
        elapsed_time = time.time() - prev_time

        if elapsed_time >= 1.0:  # Update FPS every second
            print(f"\nFPS: {frame_count}")
            last_frame_count = frame_count
            frame_count = 1
            prev_time = time.time()


        