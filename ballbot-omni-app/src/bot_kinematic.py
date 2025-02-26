"""
Bot Kinematics Module
==========================

This program takes the bot body frame torque Tx, Ty, Tz along x, y, z axes respectively
and covert it to the torques of each motor T1, T2, T3.

Authors: Yilin Ma, and Wenzhe Tong
Hybrid Dynamic Robotics Lab
"""

import numpy as np

# Constants
FREQ = 200  # Frequency in Hz
DT = 1 / FREQ  # Time step
RK = 0.1192 # Ball radius (m)
RW = 0.0488  # Wheel radius (m)
ALPHA = np.deg2rad(45)  # Motor inclination angle (radians)

# Function to convert frame torque to motor torque
"""

Warning:
This function should only be called if the bot is running on the ground!

"""
def compute_robot_motor_torques(Tx, Ty, Tz):
    coeff = -0.1
    d, r = RK * np.sin(ALPHA) ,RW
    H = 1/r * np.array([[   1,                0, -d],
                        [-1/2, -np.sin(np.pi/3), -d],
                        [-1/2,  np.sin(np.pi/3), -d]
                        ])
    T123 = H @ np.array([Tx, Ty, Tz]).reshape((3,1))
    T123 = T123.flatten()
    
    T1 = coeff*T123[0]
    T2 = coeff*T123[1]
    T3 = coeff*T123[2]

    T1 = np.clip(T1, -1, 1)
    T2 = np.clip(T2, -1, 1)
    T3 = np.clip(T3, -1, 1)

    return T1, T2, T3