"""
ROB 311 - Ball-bot Sensing and Reading Demo
This program uses a soft realtime loop to enforce loop timing. Soft real time loop is a  class
designed to allow clean exits from infinite loops with the potential for post-loop cleanup operations executing.

Authors: Senthur Raj, Gray Thomas, Yves Nazon and Elliott Rouse 
Neurobionics Lab / Locomotor Control Lab
"""

import sys
import threading
import time
import numpy as np
from threading import Thread
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from loop import SoftRealtimeLoop
from DataLogger import dataLogger

FREQ = 200
DT = 1/FREQ

RW = 0.048
RK = 0.1210
ALPHA = np.deg2rad(45)

def compute_motor_torques(Tx, Ty, Tz):
    
    T1 = (1 / 3) * (Tz - (2 * Ty) / np.cos(ALPHA))
    T2 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (-np.sqrt(3) * Tx + Ty))
    T3 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (np.sqrt(3) * Tx + Ty))


    return T1, T2, T3

def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

if __name__ == "__main__":
    trial_num = int(input('Trial Number? '))
    filename = 'ROB311_Test%i' % trial_num
    dl = dataLogger(filename + '.txt')

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    # Init serial
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Local structs
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    commands['start'] = 1.0

    dpsi = np.zeros((3, 1))

    # Time for comms to sync
    time.sleep(1.0)

    ser_dev.send_topic_data(101, commands)

    print('Beginning program!')
    i = 0

   
   
    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            states = ser_dev.get_cur_topic_data(121)[0]
        except KeyError as e:
            continue

        if i == 0:
            print('Finished calibration\nStarting loop...')
            t_start = time.time()

        i = i + 1
        t_now = time.time() - t_start

        #dpsi[0] = states['dpsi_1']
        #dpsi[1] = states['dpsi_2']
        #dpsi[2] = states['dpsi_3']

        ser_dev.send_topic_data(101, commands)
        Tx = 0
        Ty = 0
        if t_now <= 3:
            Tz = 0.25
        
        if 3 < t_now <= 6:
            Tz = 0.5

        if 6 < t_now <= 9:
            Tz = 0.75
        if 9 < t_now <= 12:
            Tz = 1
        T1,T2,T3 = compute_motor_torques (Tx, Ty, Tz)

        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3
        ser_dev.send_topic_data(101, commands) 

        # Define variables for saving / analysis here - below you can create variables from the available states
        theta_x = (states['theta_roll'])  
        theta_y = (states['theta_pitch'])
        theta_z = (states['theta_yaw'])

        dpsi[0] = states['dpsi_1']
        dpsi[1] = states['dpsi_2']
        dpsi[2] = states['dpsi_3']

        print (dpsi[0])

        # Construct the data matrix for saving - you can add more variables by replicating the format below
        data = [i] + [t_now] + [theta_x] + [theta_y] + [theta_z] + [states['dpsi_1']] + [states['dpsi_2']] + [states['dpsi_3']]
        dl.appendData(data)

        if t_now > 15:
            break

    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)
    print("Saving data...")
    dl.writeOut()
    print("Resetting Motor Commands.")
