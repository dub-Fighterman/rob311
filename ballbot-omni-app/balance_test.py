import sys
import threading
import time
import numpy as np
from threading import Thread
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from loop import SoftRealtimeLoop
from DataLogger import dataLogger
from ps4_controller_api import PS4InputHandler
from collections import deque

FREQ = 200
DT = 1/FREQ

RW = 0.048
RK = 0.1210
ALPHA = np.deg2rad(45)

#PID value
kp = 11
ki = 0.05
kd = 0.05

desired_x = 0
desired_y = 0
error_x_sum = 0
error_y_sum = 0
old_err_x = 0
old_err_y = 0
Tx_Max = 0.5
Ty_Max = 0.5
Tz_Max = 0.25
JOYSTICK_SCALE = 32767

def compute_motor_torques(Tx, Ty, Tz):
    
    T1 = (1 / 3) * (Tz - (2 * Ty) / np.cos(ALPHA))
    T2 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (-np.sqrt(3) * Tx + Ty))
    T3 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (np.sqrt(3) * Tx + Ty))


    return T1, T2, T3

def deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value

#moving average filter
ERROR_AVG_WINDOW = 5
error_x_buffer = deque(maxlen=ERROR_AVG_WINDOW)
error_y_buffer = deque(maxlen=ERROR_AVG_WINDOW)
def moving_average(buffer, new_value):
    buffer.append(new_value)
    return sum(buffer) / len(buffer)



def register_topics(ser_dev:SerialProtocol):
    # Mo :: Commands, States
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

if __name__ == "__main__":
    trial_num = int(input('Trial Number? '))
    filename = 'ROB311_Test%i' % trial_num
    dl = dataLogger(filename + '.txt')

    # === Controller Initialization ===
    # Create an instance of the PS4 controller handler
    controller = PS4InputHandler(interface="/dev/input/js0")

    # Start a separate thread to listen for controller inputs
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True  # Ensures the thread stops with the main program
    controller_thread.start()

    print("PS4 Controller is active. Use thumbsticks and triggers to control torque.")

    ser_dev = SerialProtocol()
    register_topics(ser_dev)

    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

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

            signals = controller.get_signals()
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

        #T1,T2,T3 = compute_motor_torques (Tx, Ty, Tz)

        """
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3
        ser_dev.send_topic_data(101, commands)
        """ 

        # Define variables for saving / analysis here - below you can create variables from the available states
        theta_x = (states['theta_roll'])
        theta_y = (states['theta_pitch'])
        theta_z = (states['theta_yaw'])

        dpsi[0] = states['dpsi_1']
        dpsi[1] = states['dpsi_2']
        dpsi[2] = states['dpsi_3']

        error_x = desired_x - theta_x
        error_y = desired_y - theta_y
        '''
        raw_error_x = desired_x - theta_x
        raw_error_y = desired_y - theta_y
        error_x = moving_average(error_x_buffer, raw_error_x)
        error_y = moving_average(error_y_buffer, raw_error_y)
        '''
        error_x_sum = error_x_sum + error_x
        error_y_sum = error_y_sum + error_y
        err_diff_x = error_x - old_err_x
        err_diff_y = error_y - old_err_y
        dedk_x = (moving_average(error_x_buffer, err_diff_x)) / DT
        dedk_y = (moving_average(error_y_buffer, err_diff_y)) / DT

        #pid for x
        upx = kp * error_x * -1
        uix = ki * error_x_sum * DT
        udx = kd * dedk_x

        #pid for y
        upy = kp * error_y * -1
        uiy = ki * error_y_sum *DT
        udy = kd * dedk_y

        """
        #controller input
        x_control = signals["left_thumbstick_x"] * Tx_Max
        y_control = signals["left_thumbstick_y"] * Ty_Max
        z_control = (signals["trigger_R2"] - signals["trigger_L2"]) * Tz_Max
        """      
        
        #balance PID
        Tx_balance = ((upx + uix + udx) * -1)
        Ty_balance = upy + uiy + udy
        Tz_balance = 0

        #deadband filter
        deadband_threshold = 0.125
        Tx_balance = deadband(Tx_balance, deadband_threshold)
        Ty_balance = deadband(Ty_balance, deadband_threshold)

        old_err_x = error_x
        old_err_y = error_y

        #steering trigger
        if abs(Tx_balance) <=0.4 and abs(Ty_balance) <= 0.4:
            Tx_control = signals["left_thumbstick_x"] * Tx_Max
            Ty_control = signals["left_thumbstick_y"] * Ty_Max
            Tz_control = (signals["trigger_R2"] - signals["trigger_L2"]) * Tz_Max
        else:
            Tx_control = 0
            Ty_control = 0
            Tz_control = 0

        #balance + sterring
        Tx = Tx_balance + Tx_control
        Ty = Ty_balance + Ty_control
        Tz = Tz_balance + Tz_control

        #compute torque
        T1,T2,T3 = compute_motor_torques (Tx, Ty, Tz)

        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3
        ser_dev.send_topic_data(101, commands)

        #debug print
        
        print(
            f"Time: {t_now:.2f}s | Tx: {Tx:.2f}, Ty: {Ty:.2f}, Tz: {Tz:.2f} | "
            f"T1: {T1:.2f}, T2: {T2:.2f}, T3: {T3:.2f} | "
            f"x_control: {Tx_control:.2f}, y_control: {Ty_control:.2f} , z_control: {Tz_control:.2f}"
            )
        
        '''
        #print (dpsi[0])
        print(
            f"Time: {t_now:.2f}s | upx: {upx:.2f}, uix: {uix:.2f}, udx: {udx:.2f} | "
            f"upy: {upy:.2f}, uiy: {uiy:.2f}, udy: {udy:.2f} | "
            )
            '''

        # Construct the data matrix for saving - you can add more variables by replicating the format below
        #data = [i] + [t_now] + [theta_x] + [theta_y] + [theta_z] + [states['dpsi_1']] + [states['dpsi_2']] + [states['dpsi_3']]
        data = [i] + [t_now] + [upx] + [uix] + [udx] + [upy] + [uiy] + [udy]
        #data = [i] + [t_now] + [Tx] + [Ty] + [T1] + [T2] + [T3]
        dl.appendData(data)

        ##if t_now > 15:
        ##    break


    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)
    print("Saving data...")
    dl.writeOut()
    print("Resetting Motor Commands.")