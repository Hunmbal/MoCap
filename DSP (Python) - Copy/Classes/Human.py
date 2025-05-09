

class Human:
    
    def __init__(self, temp) -> None:
        from .BODYPART import BODYPART
        self._state = "failed"
        # Center parts
        self.chest = BODYPART("chest", type=temp)
        # self.head = BODYPART("head", self.chest, type)

        # # Upper l
        self.l_shoulder = BODYPART("l_shoulder", type=temp)
        self.l_elbow = BODYPART("l_elbow", self.l_shoulder, type=temp)
        # # self.l_wrist = BODYPART()

        # # Upper r
        self.r_shoulder = BODYPART("r_shoulder", type=temp)
        self.r_elbow = BODYPART("r_elbow", self.r_shoulder, type=temp)
        # # self.r_wrist = BODYPART()

        # # Lower l
        # # self.l_hip = BODYPART("l_hip")
        # # self.l_knee = BODYPART("l_knee", self.l_hip)
        # # self.l_ankle = BODYPART("l_knee", self.l_knee)
        
        # # Lower r
        # # self.r_hip = BODYPART("r_hip")
        # # self.r_knee = BODYPART("r_knee", self.r_hip)
        # # self.r_ankle = BODYPART("r_ankle", self.r_knee)

    def get_joint_angles(self, joint_name):
        return getattr(self, joint_name).angles  # Or your actual angle storage mechanism

    def __update_data__(self, data, isCalibrating: bool):
        from .BODYPART import BODYPART
        #print("Updating data...")
        #print(f"Data received: {data}")  # Add this line to see the data format
        for name, values in data.items():
            if hasattr(self, name):
                ax, ay, az, gx, gy, gz, mx, my, mz, q0, q1, q2, q3 = values
                body_part: BODYPART = getattr(self, name)
                body_part.collectCurrentData(ax, ay, az, gx, gy, gz, mx, my, mz, q0, q1, q2, q3, isCalibrating)



    def __updateHumanOrigin__(self, pose_name):
        from .BODYPART import BODYPART
        for part_name in [attr for attr in dir(self) if not attr.startswith('_')]:
            body_part = getattr(self, part_name)
            if isinstance(body_part, BODYPART):
                body_part._calculateOriginOrientation(pose_name)


    def __print_data__(self):
        from .BODYPART import BODYPART
        for part_name in [attr for attr in dir(self) if not attr.startswith('_')]:
            body_part: BODYPART = getattr(self, part_name)
            if isinstance(body_part, BODYPART):
                print(f"{part_name:<15} -> Pitch: {body_part.pitch:<6} Roll: {body_part.roll:<6} Yaw: {body_part.yaw:<6}")

    
    def __setAbs__(self, joint_data: dict):
        from .BODYPART import BODYPART
        for name, (yaw, pitch, roll) in joint_data.items():
            if hasattr(self, name):
                body_part: BODYPART = getattr(self, name)
                body_part.origin_yaw = yaw
                body_part.origin_pitch = pitch
                body_part.origin_roll = roll

