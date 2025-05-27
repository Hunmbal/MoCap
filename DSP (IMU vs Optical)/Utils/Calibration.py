from collections import defaultdict
import os
import yaml
import time
from Classes.BODYPART import BODYPART
from Classes.Human import Human
from Classes.States import CalibrationState
import Utils.ChatBot as Jarvis
import Utils.Esp32 as Esp32

# Calibration.py
CALIBRATION_POSES = {
    'Tpose': {'file': 'files/Tpose.yml', 'instruction': 'Stand in T-pose (arms horizontal)'},
    'Ipose': {'file': 'files/Ipose.yml', 'instruction': 'Stand in I-pose (arms vertical)'},
    'Lpose': {'file': 'files/Lpose.yml', 'instruction': 'Stand in L-pose (arms pointing forward)'}
}




def check_and_calibrate(entity: Human):
    for pose_name, config in CALIBRATION_POSES.items():
        cal_file = config['file']
        
        if not os.path.exists(cal_file):
            Jarvis.say(f"{pose_name} calibration not found. Let's calibrate {pose_name}!")
            perform_pose_calibration(entity, pose_name, config)
        else:
            if Jarvis.ask_and_get(f"Found existing {pose_name} calibration. Recalibrate?"):
                perform_pose_calibration(entity, pose_name, config)
            else:
                load_pose_calibration(entity, cal_file)
                # Jarvis.say(f"Using existing {pose_name} calibration")

    entity._state = CalibrationState.COMPLETED
    Jarvis.say("All pose calibrations complete!")





def perform_pose_calibration(entity: Human, pose_name: str, config: dict):
    Jarvis.say(config['instruction'])
    entity._state = pose_name
    
    # Countdown and calibration process
    for i in range(3, 0, -1):
        Jarvis.say(str(i))
        time.sleep(1)
        
    Jarvis.say("Hold position...")
    start_time = time.time()
    
    raw_data = []
    while time.time() - start_time < 5:
        sensor_data = Esp32.read_esp32()
        raw_data.append(sensor_data)
        entity.__update_data__(sensor_data, True)
        time.sleep(0.1)

    entity.__updateHumanOrigin__(pose_name)

    # Initialize
    stats = defaultdict(lambda: defaultdict(list))

    # Fill data
    for data in raw_data:
        for bone, values in data.items():
            keys = ['ax','ay','az','gx','gy','gz','mx','my','mz','q0','q1','q2','q3']
            for k, v in zip(keys, values):
                stats[bone][k].append(v)

    # Compute min/max
    min_max = {}
    for bone, sensors in stats.items():
        min_max[bone] = {}
        for k, vals in sensors.items():
            min_max[bone][f'min_{k}'] = min(vals)
            min_max[bone][f'max_{k}'] = max(vals)



    save_pose_calibration(entity, config['file'], pose_name, min_max)





def save_pose_calibration(entity: Human, filename: str, pose_name: str, min_max):
    calibration_data = {}

    for part_name in [attr for attr in dir(entity) if not attr.startswith('_')]:
        body_part = getattr(entity, part_name)
        if isinstance(body_part, BODYPART):
            pose_data = getattr(body_part, pose_name)
            if pose_data:
                calibration_data[part_name] = {
                    'pose': {k: float(v) for k, v in pose_data.items()},
                    'min_max': min_max.get(part_name, {})
                }

    with open(filename, 'w') as f:
        yaml.dump(calibration_data, f)






def load_pose_calibration(entity: Human, filename: str):
    pose_name = os.path.splitext(os.path.basename(filename))[0]

    with open(filename, 'r') as f:
        data = yaml.safe_load(f)

    for part_name, values in data.items():
        if hasattr(entity, part_name):
            body_part: BODYPART = getattr(entity, part_name)
            pose = values.get("pose", {})
            min_max = values.get("min_max", {})
            body_part.set_pose_calibration(pose_name, **pose, min_max=min_max)
            body_part._calcMatrix()
