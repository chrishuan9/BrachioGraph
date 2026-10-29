"""MicroPython port of the BrachioGraph shoulder-and-elbow drawing robot class."""

import math
from plotter import Plotter


class BrachioGraph(Plotter):
    """A shoulder-and-elbow drawing robot class."""

    def __init__(
        self,
        virtual=False,  # a virtual plotter runs in software only
        turtle=False,  # not available on MicroPython
        turtle_coarseness=None,
        #  ----------------- geometry of the plotter -----------------
        bounds=[-8, 4, 6, 13],  # the maximum rectangular drawing area
        inner_arm=8,  # the lengths of the arms
        outer_arm=8,
        #  ----------------- naive calculation values -----------------
        servo_1_parked_pw=1500,  # pulse-widths when parked
        servo_2_parked_pw=1500,
        servo_1_degree_ms=-10,  # microseconds pulse-width per degree
        servo_2_degree_ms=10,  # reversed for the mounting of the shoulder servo
        servo_1_parked_angle=-90,  # the arm angle in the parked position
        servo_2_parked_angle=90,
        #  ----------------- hysteresis -----------------
        hysteresis_correction_1=0,  # hardware error compensation
        hysteresis_correction_2=0,
        #  ----------------- servo angles and pulse-widths in lists -----------------
        servo_1_angle_pws=[],  # pulse-widths for various angles
        servo_2_angle_pws=[],
        #  ----------------- servo angles and pulse-widths in lists (bi-directional) ------
        servo_1_angle_pws_bidi=[],  # bi-directional pulse-widths for various angles
        servo_2_angle_pws_bidi=[],
        #  ----------------- the pen -----------------
        pw_up=1500,  # pulse-widths for pen up/down
        pw_down=1100,
        #  ----------------- physical control -----------------
        wait=None,  # default wait time between operations
        angular_step=None,  # default step of the servos in degrees
        resolution=None,  # default resolution of the plotter in cm
        #  ----------------- ESP32 GPIO pins -----------------
        servo_1_pin=25,  # shoulder (was GPIO 14 on the Pi)
        servo_2_pin=26,  # elbow (was GPIO 15 on the Pi)
        pen_pin=27,  # pen lift (was GPIO 18 on the Pi)
        pwm_frequency=50,
    ):

        # set the geometry
        self.inner_arm = inner_arm
        self.outer_arm = outer_arm

        # Set the x and y position state, so it knows its current x/y position.
        self.x = -self.inner_arm
        self.y = self.outer_arm

        super().__init__(
            bounds=bounds,
            servo_1_parked_pw=servo_1_parked_pw,
            servo_2_parked_pw=servo_2_parked_pw,
            servo_1_degree_ms=servo_1_degree_ms,
            servo_2_degree_ms=servo_2_degree_ms,
            servo_1_parked_angle=servo_1_parked_angle,
            servo_2_parked_angle=servo_2_parked_angle,
            hysteresis_correction_1=hysteresis_correction_1,
            hysteresis_correction_2=hysteresis_correction_2,
            servo_1_angle_pws=servo_1_angle_pws,
            servo_2_angle_pws=servo_2_angle_pws,
            servo_1_angle_pws_bidi=servo_1_angle_pws_bidi,
            servo_2_angle_pws_bidi=servo_2_angle_pws_bidi,
            pw_up=pw_up,
            pw_down=pw_down,
            wait=wait,
            angular_step=angular_step,
            resolution=resolution,
            virtual=virtual,
            turtle=turtle,
            turtle_coarseness=turtle_coarseness,
            servo_1_pin=servo_1_pin,
            servo_2_pin=servo_2_pin,
            pen_pin=pen_pin,
            pwm_frequency=pwm_frequency,
        )

    def test_arcs(self):
        self.park()
        elbow_angle = 120
        self.move_angles(angle_2=elbow_angle)

        for angle_1 in range(-135, 15, 15):
            self.move_angles(angle_1=angle_1, draw=True)

            for angle_2 in range(elbow_angle, elbow_angle + 16):
                self.move_angles(angle_2=angle_2, draw=True)
            for angle_2 in range(elbow_angle + 16, elbow_angle - 16, -1):
                self.move_angles(angle_2=angle_2, draw=True)
            for angle_2 in range(elbow_angle - 16, elbow_angle + 1):
                self.move_angles(angle_2=angle_2, draw=True)

    # ----------------- trigonometric methods -----------------

    def xy_to_angles(self, x=0, y=0):
        """Return the servo angles required to reach any x/y position."""

        hypotenuse = math.sqrt(x**2 + y**2)

        if hypotenuse > self.inner_arm + self.outer_arm:
            raise ValueError(
                "Cannot reach {}; total arm length is {}".format(
                    hypotenuse, self.inner_arm + self.outer_arm
                )
            )

        hypotenuse_angle = math.asin(x / hypotenuse)

        inner_angle = math.acos(
            _clamp(
                (hypotenuse**2 + self.inner_arm**2 - self.outer_arm**2)
                / (2 * hypotenuse * self.inner_arm)
            )
        )
        outer_angle = math.acos(
            _clamp(
                (self.inner_arm**2 + self.outer_arm**2 - hypotenuse**2)
                / (2 * self.inner_arm * self.outer_arm)
            )
        )

        shoulder_motor_angle = hypotenuse_angle - inner_angle
        elbow_motor_angle = math.pi - outer_angle

        return (math.degrees(shoulder_motor_angle), math.degrees(elbow_motor_angle))

    def angles_to_xy(self, shoulder_motor_angle, elbow_motor_angle):
        """Return the x/y co-ordinates represented by a pair of servo angles."""

        elbow_motor_angle = math.radians(elbow_motor_angle)
        shoulder_motor_angle = math.radians(shoulder_motor_angle)

        hypotenuse = math.sqrt(
            (
                self.inner_arm**2
                + self.outer_arm**2
                - 2 * self.inner_arm * self.outer_arm * math.cos(math.pi - elbow_motor_angle)
            )
        )
        base_angle = math.acos(
            _clamp(
                (hypotenuse**2 + self.inner_arm**2 - self.outer_arm**2)
                / (2 * hypotenuse * self.inner_arm)
            )
        )
        inner_angle = base_angle + shoulder_motor_angle

        x = math.sin(inner_angle) * hypotenuse
        y = math.cos(inner_angle) * hypotenuse

        return (x, y)

    # ----------------- reporting methods -----------------

    def report(self):

        sep = "               -----------------|-----------------"
        print(sep)
        print("               Servo 1          |  Servo 2        ")
        print(sep)

        h1, h2 = self.hysteresis_correction_1, self.hysteresis_correction_2
        print("hysteresis                 {:>2.1f}  |              {:>2.1f}".format(h1, h2))

        pw_1, pw_2 = self.get_pulse_widths()
        print("pulse-width               {:<4.0f}  |             {:<4.0f}".format(pw_1, pw_2))

        angle_1, angle_2 = self.angle_1, self.angle_2

        if angle_1 and angle_2:
            print("      angle               {:>4.0f}  |             {:>4.0f}".format(angle_1, angle_2))

        print(sep)
        print("               min   max   mid  |  min   max   mid")
        print(sep)

        if (
            self.angles_used_1
            and self.angles_used_2
            and self.pulse_widths_used_1
            and self.pulse_widths_used_2
        ):
            fmt = "{}  {:>4.0f}  {:>4.0f}  {:>4.0f}  | {:>4.0f}  {:>4.0f}  {:>4.0f}"

            for label, used_1, used_2 in (
                ("pulse-widths", self.pulse_widths_used_1, self.pulse_widths_used_2),
                ("      angles", self.angles_used_1, self.angles_used_2),
            ):
                min1, max1 = min(used_1), max(used_1)
                min2, max2 = min(used_2), max(used_2)
                print(fmt.format(label, min1, max1, (min1 + max1) / 2, min2, max2, (min2 + max2) / 2))

        else:
            print("No data recorded yet. Try calling the BrachioGraph.box() method first.")


def _clamp(v):
    # guard acos() against tiny floating-point overshoots (single-precision floats on the ESP32)
    return -1.0 if v < -1.0 else (1.0 if v > 1.0 else v)
