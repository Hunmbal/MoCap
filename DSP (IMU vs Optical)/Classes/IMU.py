import math
import numpy as np
import quaternion  # Requires numpy-quaternion package
from ahrs.filters import Madgwick, Complementary
import tensorflow as tf
import joblib



from .Vector3d import Vector3D
from math import isnan


nn_model = joblib.load('mlp_model.pkl')

class IMUQ:
    """Quaternion-only orientation calculation"""
    
    @staticmethod
    def calculate(q0: float, q1: float, q2: float, q3: float):
        """
        Calculate orientation from quaternion (q0 = w component)
        Returns: yaw, pitch, roll in degrees
        """
        # # Normalize the quaternion to avoid numerical instability
        # magnitude = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        # if magnitude == 0 or isnan(magnitude):
        #     # If magnitude is zero or NaN, return default values
        #     return 0, 0, 0

        # q0 /= magnitude
        # q1 /= magnitude
        # q2 /= magnitude
        # q3 /= magnitude

        # Create a quaternion object
        q = np.quaternion(q0, q1, q2, q3)

        # Convert to Euler angles (using 'zyx' convention common in IMUs)
        try:
            roll, pitch, yaw = quaternion.as_euler_angles(q)
        except Exception as e:
            # Handle conversion errors gracefully
            print(f"Error during quaternion conversion: {e}")
            return 0, 0, 0

        # Check for NaN values and replace them with 0
        if isnan(yaw) or isnan(pitch) or isnan(roll):
            yaw = pitch = roll = 0

        # Convert to degrees and round the results
        return (
            round(np.degrees(yaw)),
            round(np.degrees(pitch)),
            round(np.degrees(roll))
        )
    




class IMU6:
    """6DOF using Complementary Filter from AHRS library"""
    
    def __init__(self, freq: float = 100):
        self.filter = Complementary(frequency=freq)
        self.q = np.array([1., 0., 0., 0.])  # Initial quaternion
        
    def calculate(self, accel: Vector3D, gyro: Vector3D, dt: float):
        gyr = np.array([gyro.x, gyro.y, gyro.z])
        acc = np.array([accel.x, accel.y, accel.z])
        
        self.q = self.filter.update(self.q, gyr=gyr, acc=acc)
        
        # Convert to Euler angles (zyx convention)
        roll = np.arctan2(2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]), 
                         1 - 2*(self.q[1]**2 + self.q[2]**2))
        pitch = np.arcsin(2*(self.q[0]*self.q[2] - self.q[3]*self.q[1]))
        yaw = np.arctan2(2*(self.q[0]*self.q[3] + self.q[1]*self.q[2]),
                       1 - 2*(self.q[2]**2 + self.q[3]**2))
        
        return (
            round(np.degrees(yaw)),
            round(np.degrees(pitch)),
            round(np.degrees(roll))
        )
    




class IMU9:
    """9DOF using Madgwick Filter from AHRS library"""
    
    def __init__(self, freq: float = 100, beta: float = 0.1):
        self.filter = Madgwick(frequency=freq, beta=beta)
        self.q = np.array([1., 0., 0., 0.])  # Initial quaternion
        
    def calculate(self, accel: Vector3D, gyro: Vector3D, mag: Vector3D, dt: float):
        gyr = np.array([gyro.x, gyro.y, gyro.z])
        acc = np.array([accel.x, accel.y, accel.z])
        mag = np.array([mag.x, mag.y, mag.z])
        
        self.q = self.filter.updateMARG(self.q, gyr=gyr, acc=acc, mag=mag)
        
        # Convert to Euler angles
        roll = np.arctan2(2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]), 
                         1 - 2*(self.q[1]**2 + self.q[2]**2))
        pitch = np.arcsin(2*(self.q[0]*self.q[2] - self.q[3]*self.q[1]))
        yaw = np.arctan2(2*(self.q[0]*self.q[3] + self.q[1]*self.q[2]),
                       1 - 2*(self.q[2]**2 + self.q[3]**2))
        
        return (
            round(np.degrees(yaw)),
            round(np.degrees(pitch)),
            round(np.degrees(roll))
        )
    


class IMUOLD:
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
    def getPitch(data):
        # accel = data.accel_history[-1]
        # pitch = np.arctan2(accel.y, accel.z)
        accel = data.accel_history[-1]
        pitch = np.arctan2(-accel.x, np.sqrt(accel.y**2 + accel.z**2))
        
        return round(np.degrees(pitch))

    @staticmethod
    def getRoll(data):
        accel = data.accel_history[-1]
        roll = np.arctan2(-accel.x, np.sqrt(accel.y**2 + accel.z**2))
        return round(np.degrees(roll))

    @staticmethod
    def getYaw(data):
        accel = data.accel_history[-1]
        mag = data.mag_history[-1]
        if np.linalg.norm(accel.to_array()) == 0:
            return 0.0
        mag_x = mag.x * np.cos(np.radians(IMUOLD.getPitch(data))) + mag.y * np.sin(np.radians(IMUOLD.getRoll(data)))
        mag_y = mag.y * np.cos(np.radians(IMUOLD.getPitch(data))) - mag.x * np.sin(np.radians(IMUOLD.getRoll(data)))
        yaw = np.arctan2(mag_y, mag_x)
        return round(np.degrees(yaw))



class IMU_CUSTOM:
    """Futher refinement of IMUQ"""
    

    def calculate(temp): 
        from Classes.BODYPART import BODYPART
        obj: BODYPART = temp
        """
        Calculate orientation from quaternion (q0 = w component)
        Returns: yaw, pitch, roll in degrees
        """

        # Create a quaternion object
        q = np.quaternion(obj.quat.w, obj.quat.x, obj.quat.y, obj.quat.z)

        # Convert to Euler angles (using 'zyx' convention common in IMUs)
        try:
            roll, pitch, yaw = quaternion.as_euler_angles(q)
        except Exception as e:
            # Handle conversion errors gracefully
            print(f"Error during quaternion conversion: {e}")
            return 0, 0, 0

        # Check for NaN values and replace them with 0
        if isnan(yaw) or isnan(pitch) or isnan(roll):
            yaw = pitch = roll = 0
        yaw = np.degrees(yaw)
        pitch = np.degrees(pitch)
        roll = np.degrees(roll)
        y, p, r = yaw, pitch, roll


        ax = np.median([v.x for v in obj.accel_history])
        ay = np.median([v.y for v in obj.accel_history])
        az = np.median([v.z for v in obj.accel_history])

        xz = np.hypot(ax,az)
        yz = np.hypot(ay,az)

        # if (obj.name.__contains__("shoulder")):

        #     if ax > 900: #I
        #         p=-85
        #         r=0
        #         y=0

        #     if az > 800: #T
        #         p=-0
        #         r=0
        #         y=0

            # if -100 < ax < 300 and yz > xz: #TL
            #     p=0
            #     r=0
            #     if (obj.name.__contains__("r_shoulder")):
            #         y += 90
            #     else:
            #         y -= 90 



        # y,p,r =0,0,0

        # p = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
        # p = -0.09 *ax

        p = -0.099*ax 
        y=0
        r=0


        # Extract and combine the 13 values into a single NumPy array



        if (obj.name.__contains__("shoulder")):
            if -45 < p < 45:
                p = -0.1163*ax +10
                if (obj.name.__contains__("r_shoulder")):
                    # # r=yaw
                    # y = roll-90
                    # y = obj.quat.y
                    # y = obj.quat.y*118+40 
                    # p = -0.1163*ax +10
                    obj2 = obj.parentbone
                    latest_data = np.array([
                        # From obj
                        obj.accel_history[-1].x,
                        obj.accel_history[-1].y,
                        obj.accel_history[-1].z,

                        obj.gyro_history[-1].x,
                        obj.gyro_history[-1].y,
                        obj.gyro_history[-1].z,

                        obj.mag_history[-1].x,
                        obj.mag_history[-1].y,
                        obj.mag_history[-1].z,

                        obj.quat_history[-1].w,
                        obj.quat_history[-1].x,
                        obj.quat_history[-1].y,
                        obj.quat_history[-1].z,

                        # From obj2 (parentbone)
                        obj2.accel_history[-1].x,
                        obj2.accel_history[-1].y,
                        obj2.accel_history[-1].z,

                        obj2.gyro_history[-1].x,
                        obj2.gyro_history[-1].y,
                        obj2.gyro_history[-1].z,

                        obj2.mag_history[-1].x,
                        obj2.mag_history[-1].y,
                        obj2.mag_history[-1].z,

                        obj2.quat_history[-1].w,
                        obj2.quat_history[-1].x,
                        obj2.quat_history[-1].y,
                        obj2.quat_history[-1].z
                    ])

                    # p = model.prodict(la)
                    p = nn_model.predict(latest_data[-1,:].reshape(1, -1))[0][0]

                else:
                    p= -0.08796*ax-5
                    # # r=yaw 
                    # y = roll-90
                    # y = obj.quat.y*118+40 
        
        if (obj.name.__contains__("elbow")):
            p = 0
            if (obj.name.__contains__("r_elbow")):
                    obj2 = obj.parentbone
                    latest_data = np.array([

                        # From obj2 (parentbone)
                        obj2.accel_history[-1].x,
                        obj2.accel_history[-1].y,
                        obj2.accel_history[-1].z,

                        obj2.gyro_history[-1].x,
                        obj2.gyro_history[-1].y,
                        obj2.gyro_history[-1].z,

                        obj2.mag_history[-1].x,
                        obj2.mag_history[-1].y,
                        obj2.mag_history[-1].z,

                        obj2.quat_history[-1].w,
                        obj2.quat_history[-1].x,
                        obj2.quat_history[-1].y,
                        obj2.quat_history[-1].z,
                        # From obj
                        obj.accel_history[-1].x,
                        obj.accel_history[-1].y,
                        obj.accel_history[-1].z,

                        obj.gyro_history[-1].x,
                        obj.gyro_history[-1].y,
                        obj.gyro_history[-1].z,

                        obj.mag_history[-1].x,
                        obj.mag_history[-1].y,
                        obj.mag_history[-1].z,

                        obj.quat_history[-1].w,
                        obj.quat_history[-1].x,
                        obj.quat_history[-1].y,
                        obj.quat_history[-1].z
                    ])

                    # p = model.prodict(la)
                    p = nn_model.predict(latest_data[-1,:].reshape(1, -1))[0][1]

            if (obj.name.__contains__("l_elbow")):
                if (-45 < obj.parentbone.pitch < 45):
                    p = -(0.12*ax +24)

        # print("LOOOL")
        # Convert to degrees and round the results
        return (
            round(y),
            round(p),
            round(r)
        )
    


