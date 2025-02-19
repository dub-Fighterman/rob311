import socket
import numpy as np
from MBot.Messages.message_defs import mo_cmds_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from ballbot_kinematics import compute_motor_torques
from loop import SoftRealtimeLoop

# Raspberry Pi server settings
PORT = 5005
BUFFER_SIZE = 1024

# Create UDP socket to listen for commands
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

print(f"Listening for commands on UDP port {PORT}...")

# Initialize Serial Communication
ser_dev = SerialProtocol()
ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]

# Define max torques
TX_MAX, TY_MAX, TZ_MAX = 2.0, 2.0, 1.0

commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
commands["start"] = 1.0  # Activate motors

for t in SoftRealtimeLoop(dt=1 / 200, report=True):
    try:
        # Receive keyboard input data from the laptop
        data, addr = sock.recvfrom(BUFFER_SIZE)
        lx, ly, l2, r2 = map(float, data.decode().split(","))

        # Map input to torques
        Tx = lx * TX_MAX
        Ty = ly * TY_MAX
        Tz = (r2 - l2) * TZ_MAX

        # Compute motor torques
        T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)

        # Send computed torques to the robot
        commands["motor_1_duty"] = T1
        commands["motor_2_duty"] = T2
        commands["motor_3_duty"] = T3
        ser_dev.send_topic_data(101, commands)

        print(f"Received: Tx={Tx:.2f}, Ty={Ty:.2f}, Tz={Tz:.2f}")

    except Exception as e:
        print(f"Error: {e}")

