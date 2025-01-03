"""
ROB 311- PS4 Controller class

Add these code snippets to your balance controller
To use the torque commands from the PS4 controller, set the torques to values obtained
from button presses. This script will print when some buttons are pressed and serves as an example
using the PS4 controller.  This script also has three example methods for commanding torque using 
the PS4 controller (rob311_bt_controller.tz_demo_X), which can be used to set the Tz torque in your
balance script.

The soft real-time loop is already included in your balance control script.
Authors: Senthur Raj, Gray Thomas, and Elliott Rouse
"""

import sys
import threading
import numpy as np
from pyPS4Controller.controller import Controller
from loop import SoftRealtimeLoop

JOYSTICK_SCALE = 32767

FREQ = 200
DT = 1/FREQ

class ROB311BTController(Controller):
    def __init__(self, interface, connecting_using_ds4drv=False, event_definition=None, event_format=None):
        super().__init__(interface, connecting_using_ds4drv, event_definition, event_format)

        # ------------------------------------
        # Declare required attributes/values 

        # DEMO 1: Modifying Tz value with Triggers
        self.tz_demo_1 = 0.0

        # DEMO 2: Modifying Tz value with Right Thumbstick (UP/DOWN)
        self.tz_demo_2 = 0.0

        # DEMO 3: Modifying Tz value with Shoulder/Bumper Buttons
        self.tz_demo_3 = 0

        # ------------------------------------

    # Continuous value with Triggers

    def on_R2_press(self, value):
        # Normalizing raw values from [-1.0, 1.0] to [0.0, 1.0]
        self.tz_demo_1 = (1.0 + np.abs(value/JOYSTICK_SCALE))/2.0

    def on_R2_release(self):
        # Reset values
        self.tz_demo_1 = 0.0

    def on_L2_press(self, value):
        # Normalizing raw values from [-1.0, 1.0] to [-1.0, 0.0]
        self.tz_demo_1 = -1 * (1.0 + np.abs(value/JOYSTICK_SCALE))/2.0

    def on_L2_release(self):
        # Reset values
        self.tz_demo_1 = 0.0

    # ----------------------------------------
    # Continuous value with Right Thumbstick (UP/DOWN)

    def on_R3_up(self, value):
        # Inverting y-axis value
        self.tz_demo_2 = -1.0 * value/JOYSTICK_SCALE

    def on_R3_down(self, value):
        # Inverting y-axis value
        self.tz_demo_2 = -1.0 * value/JOYSTICK_SCALE

    def on_R3_y_at_rest(self):
        self.tz_demo_2 = 0.0

    # ----------------------------------------
    # Integer tz_demo_3s

    def on_R1_press(self):
        print("R1 button pressed!")
        self.tz_demo_3 += 1

    def on_R1_release(self):
        pass

    def on_L1_press(self):
        print("L1 button pressed!")
        self.tz_demo_3 -= 1

    def on_L1_release(self):
        pass

    # ----------------------------------------

    def on_options_press(self):
        print("Exiting PS4 controller thread.")
        sys.exit()

if __name__ == "__main__":

    # Starting a separate thread for the BT-Controller <<>> RPi communication
    # Press "Options" button to exit this thread

    rob311_bt_controller = ROB311BTController(interface="/dev/input/js0")
    rob311_bt_controller_thread = threading.Thread(target=rob311_bt_controller.listen, args=(10,))
    rob311_bt_controller_thread.start()

    # The "rob311_bt_controller" object has Tz attributes that can be called/used anywhere within the main block--these are potential ways to control
    # vertical axis torque using different commands from the PS4 controller
    # You can also create your own variable (e.g. rob311_bt_controller.tz) and use the button commands to create your own torque command using the button presses

    # DEMO 1: Modifying Tz value with Triggers
    # DEMO 2: Modifying Tz value with Right Thumbstick (UP/DOWN)
    # DEMO 3: Modifying Tz value with Shoulder/Bumper Buttons

    for t in SoftRealtimeLoop(dt=DT, report=True):
        print("\n\nTz Demo 1: {}\nTz Demo 2: {}\nTz Demo 3: {}\n\n".format(
            rob311_bt_controller.tz_demo_1,
            rob311_bt_controller.tz_demo_2,
            rob311_bt_controller.tz_demo_3
        ))