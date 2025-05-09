import time
import numpy as np
import quaternion
from collections import deque
from Classes.IMU import IMU6, IMU9, IMU_CUSTOM, IMUQ
from .Vector3d import Vector3D
from Classes.Poses import L_data, T_data, I_data

# Configuration ================================================================
METHOD = 2
CALM = "I"
# ===============================================================================

MAX_HISTORY_SIZE = 20
SAMPLING_RATE = 30.0  # Hz, should match your actual sampling rate


class BODYPART:
    def __init__(self, name, parent=None, type='IMUQ'):

        self.name = name
        self.IMU_TYPE = type

        # Parent bone to inherit angles as well
        self.parentbone = parent 
        self.mat = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 0]
        ])

        # Sensor history buffers
        self.accel_history = deque(maxlen=MAX_HISTORY_SIZE)
        self.gyro_history = deque(maxlen=MAX_HISTORY_SIZE)
        self.mag_history = deque(maxlen=MAX_HISTORY_SIZE)
        self.quat_history = deque(maxlen=MAX_HISTORY_SIZE)
        self.timestamps = deque(maxlen=MAX_HISTORY_SIZE)

        # Orientation state
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.quat = np.quaternion(1, 0, 0, 0)

        # Calibration state
        self.Tpose = {'yaw': 0, 'pitch': 0, 'roll': 0}
        self.Ipose = {'yaw': 0, 'pitch': 0, 'roll': 0} 
        self.Lpose = {'yaw': 0, 'pitch': 0, 'roll': 0}
        self.Tpose_min = {}
        self.Tpose_max = {}
        self.Ipose_min = {}
        self.Ipose_max = {}
        self.Lpose_min = {}
        self.Lpose_max = {}
        # self.origin_quat = np.quaternion(1, 0, 0, 0)
    

        # IMU processor initialization
        self.imu_processor = self._create_imu_processor()



    def _create_imu_processor(self):
        if self.IMU_TYPE == 'IMUQ':
            return IMUQ()
        elif self.IMU_TYPE == 'IMU6':
            return IMU6(freq=SAMPLING_RATE)
        elif self.IMU_TYPE == 'IMU9':
            return IMU9(freq=SAMPLING_RATE)
        elif self.IMU_TYPE == 'CUSTOM':
            return IMU_CUSTOM()
        raise ValueError(f"Invalid self.IMU_TYPE: {self.IMU_TYPE}")




    def collectCurrentData(self, ax: float, ay: float, az: float, 
                         gx: float, gy: float, gz: float,
                         mx: float, my: float, mz: float,
                         q0: float, q1: float, q2: float, q3: float, 
                         isCalib: bool):
        # Store timestamp first
        self.timestamps.append(time.time())
        
        # Store calibrated data
        accel, gyro, mag = self._calibrate_data(ax, ay, az, gx, gy, gz, mx, my, mz)

        self.accel_history.append(accel)
        self.gyro_history.append(gyro)
        self.mag_history.append(mag)

        self.quat = np.quaternion(q0, q1, q2, q3)
        self.quat_history.append(self.quat)

        # Calculate orientation
        if not isCalib:
            self._calculate_current_orientation()





    def _calibrate_data(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """Apply calibration offsets here"""
        # Replace with actual calibration math
        #TODO
        return (
            Vector3D(ax, ay, az),
            Vector3D(gx, gy, gz),
            Vector3D(mx, my, mz)
        )




    def _calculate_current_orientation(self):
        """Calculate orientation using selected IMU type"""
        dt = self._get_time_delta()


        syaw, spitch, sroll = IMUQ.calculate(
            self.quat.w, self.quat.x, self.quat.y, self.quat.z
        )




        # bone ypr after origin correction
        byaw, bpitch, broll = self.sensorToBone(syaw, spitch, sroll)
        
        # taking parent bone into account as child bone inherits the angles
        if self.parentbone:
            self.yaw = self._wrap_angle(byaw - self.parentbone.yaw)
            self.pitch = self._wrap_angle(bpitch - self.parentbone.pitch)
            self.roll = self._wrap_angle(broll - self.parentbone.roll)
        else:
            self.yaw = self._wrap_angle(byaw)
            self.pitch = self._wrap_angle(bpitch)
            self.roll = self._wrap_angle(broll)

        if self.IMU_TYPE == "CUSTOM" and self.name != "chest":
            self.yaw , self.pitch, self.roll = IMU_CUSTOM.calculate(temp=self)




    def _calculateOriginOrientation(self, pose_name):
        """Calculate origin offset using the same IMU processor as current data"""

        dt = self._get_time_delta()

        if not any([self.accel_history, self.quat_history]):
            return  # No calibration data available
        
        orientations = []
        
        # Process all calibration data points
        for i in range(len(self.accel_history)):
            try:
                q = self.quat_history[i]
                ypr = IMUQ.calculate(q.w, q.x, q.y, q.z)
                orientations.append(ypr)
            except IndexError:
                break  # Handle uneven calibration data
        
        if orientations:
            # Calculate median orientation (more robust than mean)
            yaws, pitches, rolls = zip(*orientations)
            y = np.median(yaws)
            p = np.median(pitches)
            r = np.median(rolls)
            
            # For quaternion-based systems, also average quaternions
            if self.IMU_TYPE == 'IMUQ' and self.quat_history:
                self.origin_quat = np.mean(self.quat_history)

            if pose_name == "Tpose":
                self.Tpose = {'yaw': y, 'pitch': p, 'roll': r}
            elif pose_name == "Ipose":
                self.Ipose = {'yaw': y, 'pitch': p, 'roll': r}
            elif pose_name == "Lpose":
                self.Lpose = {'yaw': y, 'pitch': p, 'roll': r}




    def _get_time_delta(self):
        """Calculate actual time delta between samples"""
        if len(self.timestamps) < 2:
            return 1.0 / SAMPLING_RATE
        return self.timestamps[-1] - self.timestamps[-2]





    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-180, 180) range"""
        return (angle + 180) % 360 - 180
    




    def _calcMatrix(self):
        try:
            # Get target angles with fallback
            y_tpose = list(T_data.get(self.name, (0, 0, 0)))
            y_ipose = list(I_data.get(self.name, (0, 0, 0)))
            y_lpose = list(L_data.get(self.name, (0, 0, 0)))
            
            # Build matrices
            X = np.array([
                list(self.Tpose.values()),
                list(self.Ipose.values()),
                list(self.Lpose.values())
            ])
            Y = np.array([y_tpose, y_ipose, y_lpose])
            
            # Add bias term
            Xp = np.hstack([X, np.ones((3, 1))])
            
            # Regularized least squares solution
            lambda_reg = 0.1  # Small regularization factor
            I = np.eye(Xp.shape[1])  # Identity matrix
            self.mat = np.linalg.solve(Xp.T @ Xp + lambda_reg * I, Xp.T @ Y)
            
            # Verify solution isn't all zeros
            if np.allclose(self.mat, 0):
                print(f"Warning: Zero matrix detected for {self.name}")
                self.mat = np.array([
                    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                    [0, 0, 0]  # Bias terms
                ])
                
        except Exception as e:
            print(f"Error in {self.name} matrix calculation: {str(e)}")
            self.mat = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [0, 0, 0]
            ])





    def sensorToBone(self, yaw, pitch, roll):
        if METHOD == 1:
            print(self.mat)
            x_new = np.array([yaw, pitch, roll])
            return np.append(x_new, 1) @ self.mat
        else: 
            if CALM == "T":
                y,p,r = list(self.Tpose.values())
                return  np.array([yaw-y, pitch-p,roll-r])
            elif CALM == "I":
                y,p,r = list(self.Ipose.values())
                return  np.array([yaw-y, pitch-p,roll-r])
            elif CALM == "L":
                y,p,r = list(self.Lpose.values())
                return  np.array([yaw-y, pitch-p,roll-r])







    def set_pose_calibration(self, pose_name: str, yaw: float, pitch: float, roll: float, min_max: dict = None):
        if hasattr(self, pose_name):
            getattr(self, pose_name).update({'yaw': yaw, 'pitch': pitch, 'roll': roll})
            if min_max:
                min_vals = {k: v for k, v in min_max.items() if k.startswith("min_")}
                max_vals = {k: v for k, v in min_max.items() if k.startswith("max_")}
                setattr(self, f"{pose_name}_min", min_vals)
                setattr(self, f"{pose_name}_max", max_vals)
        else:
            raise ValueError(f"Invalid pose name: {pose_name}")



    def get_pose_data(self, pose_name: str):
        if hasattr(self, pose_name):
            return getattr(self, pose_name)
        raise ValueError(f"Invalid pose name: {pose_name}")
 








