import math
import numpy as np
import quaternion  # Requires numpy-quaternion package
from ahrs.filters import Madgwick, Complementary

from .Vector3d import Vector3D
from math import isnan

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

        if (obj.name.__contains__("shoulder")):
            if -45 < p < 45:
                if (obj.name.__contains__("r_shoulder")):
                    # # r=yaw
                    # y = roll-90
                    # y = obj.quat.y
                    # y = obj.quat.y*118+40 
                    p = -0.1163*ax +10
                else:
                    p= -0.08796*ax-5
                    # # r=yaw 
                    # y = roll-90
                    # y = obj.quat.y*118+40 
        
        if (obj.name.__contains__("elbow")):
            p = 0
            if (obj.name.__contains__("r_elbow")):
                if (-45 < obj.parentbone.pitch < 45):
                    p = -(0.105*ax +5)
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
    


