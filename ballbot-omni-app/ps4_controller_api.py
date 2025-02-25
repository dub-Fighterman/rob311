"""
PS4 Input Handler API

This module provides the `PS4InputHandler` class to interface with a PS4 controller. 
It captures inputs from the left and right thumbsticks, triggers, and shoulder buttons, 
and processes them into normalized values suitable for robotics or control systems.

Author: Yilin Ma
Date: January 2025
"""

from pyPS4Controller.controller import Controller
import numpy as np

# Scale factor for normalizing joystick and trigger values
JOYSTICK_SCALE = 32767


class PS4InputHandler(Controller):
    def __init__(self, interface, **kwargs):
        """
        Initialize the PS4 controller and define the control signals.

        :param interface: The interface for the PS4 controller (e.g., "/dev/input/js0").
        """
        super().__init__(interface, **kwargs)
        self.signals = {
            "left_thumbstick_x": 0.0,   # Left thumbstick horizontal (strafe)
            "left_thumbstick_y": 0.0,   # Left thumbstick vertical (forward/backward)
            "right_thumbstick_x": 0.0,  # Right thumbstick horizontal (yaw)
            "right_thumbstick_y": 0.0,  # Right thumbstick vertical (pitch)
            "trigger_L2": 0.0,          # Left trigger (e.g., brake)
            "trigger_R2": 0.0,          # Right trigger (e.g., accelerate)
            "shoulder_L1": 0,           # Left shoulder button (decrement)
            "shoulder_R1": 0,           # Right shoulder button (increment)
        }
        self.running = True  # Control loop flag

    # === Left Thumbstick (L3) ===
    def on_L3_left(self, value):  self.signals["left_thumbstick_x"] = value / JOYSTICK_SCALE
    def on_L3_right(self, value): self.signals["left_thumbstick_x"] = value / JOYSTICK_SCALE
    def on_L3_up(self, value):    self.signals["left_thumbstick_y"] = -value / JOYSTICK_SCALE
    def on_L3_down(self, value):  self.signals["left_thumbstick_y"] = -value / JOYSTICK_SCALE
    def on_L3_x_at_rest(self):    self.signals["left_thumbstick_x"] = 0.0
    def on_L3_y_at_rest(self):    self.signals["left_thumbstick_y"] = 0.0

    # === Right Thumbstick (R3) ===
    def on_R3_left(self, value):  self.signals["right_thumbstick_x"] = value / JOYSTICK_SCALE
    def on_R3_right(self, value): self.signals["right_thumbstick_x"] = value / JOYSTICK_SCALE
    def on_R3_up(self, value):    self.signals["right_thumbstick_y"] = -value / JOYSTICK_SCALE
    def on_R3_down(self, value):  self.signals["right_thumbstick_y"] = -value / JOYSTICK_SCALE
    def on_R3_x_at_rest(self):    self.signals["right_thumbstick_x"] = 0.0
    def on_R3_y_at_rest(self):    self.signals["right_thumbstick_y"] = 0.0

    # === Triggers (L2 and R2) ===
    def on_L2_press(self, value):
        self.signals["trigger_L2"] = (value + JOYSTICK_SCALE) / (2 * JOYSTICK_SCALE)  # Normalize to [0,1]
    def on_L2_release(self): self.signals["trigger_L2"] = 0.0

    def on_R2_press(self, value):
        self.signals["trigger_R2"] = (value + JOYSTICK_SCALE) / (2 * JOYSTICK_SCALE)  # Normalize to [0,1]
    def on_R2_release(self): self.signals["trigger_R2"] = 0.0

    # === Shoulder Buttons (L1 and R1) ===
    def on_L1_press(self):  self.signals["shoulder_L1"] = 1
    def on_L1_release(self): self.signals["shoulder_L1"] = 0
    def on_R1_press(self):  self.signals["shoulder_R1"] = 1
    def on_R1_release(self): self.signals["shoulder_R1"] = 0

    # === Exit Handler ===
    def on_options_press(self):
        print("Exiting PS4 controller thread.")
        self.running = False  # Gracefully exit event loop

    # === Method to Expose Signals ===
    def get_signals(self, verbose=False):
        """
        Get the current state of all control signals.

        :param verbose: If True, prints the signals for debugging.
        :return: A dictionary containing the current values of all signals.
        """
        if verbose:
            print(self.signals)
        return self.signals

    # === Start Listening ===
    def listen(self, timeout=None):
        """
        Starts the controller's event listener.

        :param timeout: Optional timeout in seconds for listening.
        """
        print("Listening for PS4 controller input...")
        self.run(timeout=timeout)  # Starts the event loop from pyPS4Controller
