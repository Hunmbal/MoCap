from collections import deque
import time
import cv2
import mediapipe as mp
import numpy as np
import math
import socket




# MediaPipe Pose setup
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(model_complexity=1, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
CamYPR = deque(maxlen=1000)



# UDP Setup for Blender communication
serverIP = '127.0.0.1'
serverPort = 26000
clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)




# Coordinate transformation matrix to align with Blender's system
rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])




def is_landmark_valid(landmark):
    """Check if a landmark is valid based on visibility."""
    return landmark.visibility > 0.3 if hasattr(landmark, 'visibility') else True


def wrap_angle(angle):
    return ((angle + 180) % 360) - 180


def get_angles(p1, p2):
    """Calculate pitch, yaw, and roll between two points in Blender's coordinate system."""
    vec = np.dot(rotation_matrix, np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z]))
    norm = np.linalg.norm(vec)
    if norm < 1e-9:
        return (0, 0, 0)
    vec /= norm

    # Adjusted calculations for Mixamo rig
    pitch = math.degrees(math.atan2(-vec[2], vec[0]))  # X-axis rotation
    yaw = math.degrees(math.atan2(vec[1], vec[0]))     # Y-axis rotation
    roll = math.degrees(math.asin(-vec[2]))            # Z-axis rotation
    
    return (pitch, yaw, roll)


def chest(lm):
    """
    Calculate pitch, yaw, and roll for the chest (spine).
    Format the data string and return it.
    """
    # Check if required landmarks are valid
    if not all(is_landmark_valid(lm[idx]) for idx in [
        mp_pose.PoseLandmark.LEFT_HIP,
        mp_pose.PoseLandmark.RIGHT_HIP,
        mp_pose.PoseLandmark.LEFT_SHOULDER,
        mp_pose.PoseLandmark.RIGHT_SHOULDER]):
        return "mixamorig:Spine:0,0,0"  # Default if landmarks are missing

    # Compute midpoints for hips and shoulders
    hip_base = type('', (), {})()
    hip_base.x = (lm[mp_pose.PoseLandmark.LEFT_HIP].x + lm[mp_pose.PoseLandmark.RIGHT_HIP].x) / 2
    hip_base.y = (lm[mp_pose.PoseLandmark.LEFT_HIP].y + lm[mp_pose.PoseLandmark.RIGHT_HIP].y) / 2
    hip_base.z = (lm[mp_pose.PoseLandmark.LEFT_HIP].z + lm[mp_pose.PoseLandmark.RIGHT_HIP].z) / 2

    shoulder_top = type('', (), {})()
    shoulder_top.x = (lm[mp_pose.PoseLandmark.LEFT_SHOULDER].x + lm[mp_pose.PoseLandmark.RIGHT_SHOULDER].x) / 2
    shoulder_top.y = (lm[mp_pose.PoseLandmark.LEFT_SHOULDER].y + lm[mp_pose.PoseLandmark.RIGHT_SHOULDER].y) / 2
    shoulder_top.z = (lm[mp_pose.PoseLandmark.LEFT_SHOULDER].z + lm[mp_pose.PoseLandmark.RIGHT_SHOULDER].z) / 2

    # Calculate chest angles (hips to shoulders)
    chest_angles = get_angles(hip_base, shoulder_top)

    # Adjust angles for Mixamo compatibility
    chest_pitch, chest_yaw, chest_roll = (
        chest_angles[0] * -1,  # Invert pitch
        chest_angles[1],
        chest_angles[2] * -1   # Invert roll
    )

    # Format data string
    chest_data = f"mixamorig:Spine:{int(wrap_angle(-1.5*(chest_roll+67)))},{int(0)},{int(0)}"

    return chest_data


def right_arm(lm):
    """
    Calculate pitch, yaw, and roll for the right arm.
    Format the data string and return it.
    """
    # Check if landmarks are valid
    if not all(is_landmark_valid(lm[idx]) for idx in [
        mp_pose.PoseLandmark.RIGHT_SHOULDER,
        mp_pose.PoseLandmark.RIGHT_ELBOW,
        mp_pose.PoseLandmark.RIGHT_WRIST]):
        return "mixamorig:RightArm:0,0,0\nmixamorig:RightForeArm:0,0,0"  # Default if landmarks are missing

    # Extract landmarks
    shoulder = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    elbow = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
    wrist = lm[mp_pose.PoseLandmark.RIGHT_WRIST]

    # Calculate angles
    upper_arm_angles = get_angles(shoulder, elbow)
    forearm_angles = get_angles(elbow, wrist)

    # Invert pitch and roll for Mixamo compatibility
    upper_arm_pitch, upper_arm_yaw, upper_arm_roll = (
        upper_arm_angles[0] * -1,
        upper_arm_angles[1],
        upper_arm_angles[2] * -1
    )

    forearm_pitch, forearm_yaw, forearm_roll = (
        forearm_angles[0] * -1,
        forearm_angles[1],
        forearm_angles[2] * -1
    )

    # Calculate forearm angles relative to the upper arm
    f_pitch = forearm_pitch - upper_arm_pitch
    f_yaw = forearm_yaw - upper_arm_yaw
    f_roll = forearm_roll - upper_arm_roll

    # Format data strings
    upper_arm_data = f"mixamorig:RightArm:{int(wrap_angle(upper_arm_pitch-180))},{int(0)},{int(0)}"
    forearm_data = f"mixamorig:RightForeArm:{int(wrap_angle(f_pitch))},{int(0)},{int(0)}"

    return upper_arm_data + "\n" + forearm_data


def left_arm(lm):
    """
    Calculate pitch, yaw, and roll for the left arm.
    Format the data string and return it.
    """
    # Check if landmarks are valid
    if not all(is_landmark_valid(lm[idx]) for idx in [
        mp_pose.PoseLandmark.LEFT_SHOULDER,
        mp_pose.PoseLandmark.LEFT_ELBOW,
        mp_pose.PoseLandmark.LEFT_WRIST]):
        return "mixamorig:LeftArm:0,0,0\nmixamorig:LeftForeArm:0,0,0"  # Default if landmarks are missing

    # Extract landmarks
    shoulder = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
    elbow = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
    wrist = lm[mp_pose.PoseLandmark.LEFT_WRIST]

    # Calculate angles
    upper_arm_angles = get_angles(shoulder, elbow)
    forearm_angles = get_angles(elbow, wrist)

    # Invert pitch and roll for Mixamo compatibility
    upper_arm_pitch, upper_arm_yaw, upper_arm_roll = (
        upper_arm_angles[0] * -1,
        upper_arm_angles[1],
        upper_arm_angles[2] * -1
    )

    forearm_pitch, forearm_yaw, forearm_roll = (
        forearm_angles[0] * -1,
        forearm_angles[1],
        forearm_angles[2] * -1
    )

    # Calculate forearm angles relative to the upper arm
    f_pitch = forearm_pitch - upper_arm_pitch
    f_yaw = forearm_yaw - upper_arm_yaw
    f_roll = forearm_roll - upper_arm_roll

    # Format data strings
    upper_arm_data = f"mixamorig:LeftArm:{int(wrap_angle(-upper_arm_pitch))},{int(0)},{int(0)}"
    forearm_data = f"mixamorig:LeftForeArm:{int(wrap_angle(-f_pitch))},{int(0)},{int(0)}"

    return upper_arm_data + "\n" + forearm_data




name_map = {
    'mixamorig:LeftArm': 'l_shoulder',
    'mixamorig:LeftForeArm': 'l_elbow',
    'mixamorig:RightArm': 'r_shoulder',
    'mixamorig:RightForeArm': 'r_elbow',
}

def print_data():

    # Parse and format
    for line in CamYPR[-1].strip().splitlines():
        name_part, angles = line.split(":")[0] + ":" + line.split(":")[1], line.split(":")[2]
        pitch, roll, yaw = angles.split(",")
        label = name_map.get(name_part, "Unknown")
        print(f"{label:<16} -> Pitch: {pitch:<5} Roll: {roll:<6} Yaw: {yaw}")


def get_data():
    bone_rotations = CamYPR[-1]
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


# Main function
def run_pose_tracking():
    time.sleep(1)
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Windows-specific flag
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 20)
    time.sleep(2)  # Give camera more time to initialize
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    prev_time = time.time()

    while True:
        current_time = time.time()
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame - check camera connection")
            time.sleep(0.1)  # Prevent 100% CPU usage
            continue

        try:

            # Process frame
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(frame_rgb)
            
            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark
                transfer_data = right_arm(lm) + "\n" + left_arm(lm)
                clientSocket.sendto(transfer_data.encode(), (serverIP, serverPort))
                CamYPR.append(transfer_data)


                # Draw landmarks and connections
                # mp_drawing.draw_landmarks(
                #     frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                #     mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                #     mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                # )

            cv2.flip(frame, 1)


            # Calculate FPS
            fps = 1 / (current_time - prev_time)
            prev_time = current_time
            # Display FPS on the frame
            cv2.putText(frame, f"FPS: {fps:.2f}", (530, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            display_frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            # Display frame
            cv2.imshow('Pose Tracking', display_frame)
            if cv2.waitKey(1) == 27:
                break

        except Exception as e:
            print(f"Processing error: {str(e)}")
            continue

    cap.release()
    cv2.destroyAllWindows()
    cv2.destroyAllWindows()