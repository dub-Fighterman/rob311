import sys
from pyPS4Controller.controller import Controller

class PS4RawDemo(Controller):
    """
    Simple demo: print out raw values for left/right joysticks, triggers, and shoulder buttons.

    Joysticks(L3 & R3)
                     -32767
                        |
                        |
                        |
            -32767 ---- 0 ----- 32767
                        |
                        |
                        |
                      32767

    Triggers(L2 & R2)
                        ~
                        |
                        |
                        0
                        |
                        |
                        |
                      32767
    """

    def __init__(self, interface="/dev/input/js0", connecting_using_ds4drv=False):
        super().__init__(interface=interface, connecting_using_ds4drv=connecting_using_ds4drv)

    # -----------------------------------------------------------------------
    # LEFT JOYSTICK EVENTS (L3)
    # -----------------------------------------------------------------------
    def on_L3_x_axis_move(self, value):
        print(f"[L3] X-Axis raw value = {value}")

    def on_L3_y_axis_move(self, value):
        print(f"[L3] Y-Axis raw value = {value}")

    # -----------------------------------------------------------------------
    # RIGHT JOYSTICK EVENTS (R3)
    # -----------------------------------------------------------------------
    def on_R3_x_axis_move(self, value):
        print(f"[R3] X-Axis raw value = {value}")

    def on_R3_y_axis_move(self, value):
        print(f"[R3] Y-Axis raw value = {value}")

    # -----------------------------------------------------------------------
    # TRIGGERS (L2, R2)
    # -----------------------------------------------------------------------
    def on_L2_press(self, value):
        print(f"[L2] Trigger pressed, raw value = {value}")

    def on_L2_release(self):
        print("[L2] Trigger released (raw value = 0)")

    def on_R2_press(self, value):
        print(f"[R2] Trigger pressed, raw value = {value}")

    def on_R2_release(self):
        print("[R2] Trigger released (raw value = 0)")

    # -----------------------------------------------------------------------
    # SHOULDER BUTTONS (L1, R1)
    # -----------------------------------------------------------------------
    def on_L1_press(self):
        print("[L1] Shoulder button pressed")

    def on_L1_release(self):
        print("[L1] Shoulder button released")

    def on_R1_press(self):
        print("[R1] Shoulder button pressed")

    def on_R1_release(self):
        print("[R1] Shoulder button released")

def main():
    controller = PS4RawDemo(interface="/dev/input/js0", connecting_using_ds4drv=False)

    try:
        print("Starting PS4 raw demo. Move/press controls to see raw values. Press Ctrl+C to exit.")
        controller.listen()
    except KeyboardInterrupt:
        print("\nExiting PS4 raw demo...")

if __name__ == "__main__":
    main()