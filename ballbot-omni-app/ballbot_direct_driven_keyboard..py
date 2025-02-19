"""
Steering Controller for Ball-Bot on the Floor (Keyboard Version)

Replaces PS4 controller input with keyboard input.
- `WASD` for movement
- `Q/E` for rotation (yaw)
"""

import threading
import time
import keyboard  # For keyboard input
from loop import SoftRealtimeLoop
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques  # Import torque computation
from DataLogger import dataLogger  # Import data logger for logging data
import numpy as np

# Constants
FREQ = 200  # Hz
DT = 1 / FREQ  # Time step

# Torque scaling factors
TX_MAX = 2.0  # Sideways motion
TY_MAX = 2.0  # Forward/backward motion
TZ_MAX = 1.0  # Yaw (rotation)

# Function to get keyboard-based control signals
def get_keyboard_signals():
    """
    Reads keyboard inputs and maps them to control signals.
    """
    signals = {
        "left_thumbstick_x": 0.0,  # Left/Right (A/D)
        "left_thumbstick_y": 0.0,  # Forward/Backward (W/S)
        "trigger_L2": 0.0,  # Rotate Left (Q)
        "trigger_R2": 0.0,  # Rotate Right (E)
    }

    # Movement controls
    if keyboard.is_pressed("a"):
        signals["left_thumbstick_x"] = -1.0
    elif keyboard.is_pressed("d"):
        signals["left_thumbstick_x"] = 1.0

    if keyboard.is_pressed("w"):
        signals["left_thumbstick_y"] = 1.0
    elif keyboard.is_pressed("s"):
        signals["left_thumbstick_y"] = -1.0

    # Rotation controls
    if keyboard.is_pressed("q"):
        signals["trigger_L2"] = 1.0  # Rotate Left
    if keyboard.is_pressed("e"):
        signals["trigger_R2"] = 1.0  # Rotate Right

    return signals


if __name__ == "__main__":
    # === Serial Communication Initialization ===
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

    # Start serial communication thread
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # === Data Logging Initialization ===
    trial_num = int(input("Trial Number? "))
    filename = f"steering_controller_trial_{trial_num}.txt"
    dl = dataLogger(filename)

    # Define command structure for motors
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    commands["start"] = 1.0  # Activate motors

    # Initialize sensor state reading
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    # Allow communication to sync
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)

    print("Starting keyboard-controlled steering loop...")
    print("Use 'WASD' to move, 'Q' and 'E' to rotate. Press Ctrl+C to exit.")

    # === Main Control Loop ===
    i = 0  # Iteration counter
    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            # Read sensor data from the robot
            states = ser_dev.get_cur_topic_data(121)[0]

            # === Controller Input ===
            # Fetch signals from keyboard
            signals = get_keyboard_signals()

            # Map keyboard input to torques
            Tx = signals["left_thumbstick_x"] * TX_MAX
            Ty = signals["left_thumbstick_y"] * TY_MAX
            Tz = (signals["trigger_R2"] - signals["trigger_L2"]) * TZ_MAX

            # === Torque Computation ===
            T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)

            # Send computed torques to the robot
            commands["motor_1_duty"] = T1
            commands["motor_2_duty"] = T2
            commands["motor_3_duty"] = T3
            ser_dev.send_topic_data(101, commands)

            # === Data Logging ===
            if i == 0:
                t_start = time.time()  # Start time of the loop
            t_now = time.time() - t_start  # Elapsed time
            i += 1

            # Extract sensor readings
            theta_x = states["theta_roll"]  # Roll angle (x-axis)
            theta_y = states["theta_pitch"]  # Pitch angle (y-axis)
            psi_1 = states["psi_1"]  # Motor 1 rotation
            psi_2 = states["psi_2"]  # Motor 2 rotation
            psi_3 = states["psi_3"]  # Motor 3 rotation

            # Log data to the file
            data = [i, t_now, theta_x, theta_y, Tx, Ty, Tz, T1, T2, T3, psi_1, psi_2, psi_3]
            dl.appendData(data)

            # Print debug information
            print(
                f"Time: {t_now:.2f}s | Tx: {Tx:.2f}, Ty: {Ty:.2f}, Tz: {Tz:.2f} | "
                f"T1: {T1:.2f}, T2: {T2:.2f}, T3: {T3:.2f} | "
                f"Roll: {theta_x:.2f}, Pitch: {theta_y:.2f}",
                end="\r",
            )

        except KeyError:
            print("Waiting for sensor data...")

    # === Shutdown ===
    print("\nSaving data...")
    dl.writeOut()  # Write logged data to the file

    print("Shutting down motors...")
    commands["start"] = 0.0
    commands["motor_1_duty"] = 0.0
    commands["motor_2_duty"] = 0.0
    commands["motor_3_duty"] = 0.0
    ser_dev.send_topic_data(101, commands)

    print("Controller stopped.")
