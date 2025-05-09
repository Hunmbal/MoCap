import numpy as np
from Classes import BODYPART
from .Vector3d import Vector3D  # Ensure this matches the actual file name

class IMU:
    @staticmethod
    def getCalibratedData(ax: float, ay: float, az: float, gx: float, gy: float, gz: float, mx: float, my: float, mz: float):
        
        ax = (1 / 14222) * (ax + 114)
        ay = (1 / 16380) * (ay + 308)
        az = (1 / 16544) * (az + 564)

        gx = gx + 8
        gy = gy - 50
        gz = gy - 70

        return Vector3D(ax, ay, az), Vector3D(gx, gy, gz), Vector3D(mx, my, mz)

    @staticmethod
    def getPitch(data: BODYPART):
        # accel = data.accel_history[-1]
        # pitch = np.arctan2(accel.y, accel.z)
        accel = data.accel_history[-1]
        pitch = np.arctan2(-accel.x, np.sqrt(accel.y**2 + accel.z**2))
        
        return round(np.degrees(pitch))

    @staticmethod
    def getRoll(data: BODYPART):
        accel = data.accel_history[-1]
        roll = np.arctan2(-accel.x, np.sqrt(accel.y**2 + accel.z**2))
        return round(np.degrees(roll))

    @staticmethod
    def getYaw(data: BODYPART):
        accel = data.accel_history[-1]
        mag = data.mag_history[-1]
        if np.linalg.norm(accel.to_array()) == 0:
            return 0.0
        mag_x = mag.x * np.cos(np.radians(IMU.getPitch(data))) + mag.y * np.sin(np.radians(IMU.getRoll(data)))
        mag_y = mag.y * np.cos(np.radians(IMU.getPitch(data))) - mag.x * np.sin(np.radians(IMU.getRoll(data)))
        yaw = np.arctan2(mag_y, mag_x)
        return round(np.degrees(yaw))
    
    @staticmethod
    def getOriginPitch(data: BODYPART):
        accel_mean = np.mean([v.to_array() for v in data.accel_calib_vals], axis=0)
        pitch = np.arctan2(accel_mean[1], accel_mean[2])
        return round(np.degrees(pitch))

    @staticmethod
    def getOriginRoll(data: BODYPART):
        accel_mean = np.mean([v.to_array() for v in data.accel_calib_vals], axis=0)
        roll = np.arctan2(-accel_mean[0], np.sqrt(accel_mean[1]**2 + accel_mean[2]**2))
        return round(np.degrees(roll))

    @staticmethod
    def getOriginYaw(data: BODYPART):
        accel_mean = np.mean([v.to_array() for v in data.accel_calib_vals], axis=0)
        mag_mean = np.mean([v.to_array() for v in data.mag_calib_vals], axis=0)
        if np.linalg.norm(accel_mean) == 0:
            return 0.0
        mag_x = mag_mean[0] * np.cos(np.radians(data.origin_pitch)) + mag_mean[1] * np.sin(np.radians(data.origin_roll))
        mag_y = mag_mean[1] * np.cos(np.radians(data.origin_pitch)) - mag_mean[0] * np.sin(np.radians(data.origin_roll))
        yaw = np.arctan2(mag_y, mag_x)
        return round(np.degrees(yaw))