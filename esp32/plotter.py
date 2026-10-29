"""MicroPython port of the BrachioGraph ``Plotter`` base class.

Differences from the Raspberry Pi version:

* no pigpio: the servos are driven with ``machine.PWM`` at 50 Hz (``duty_ns``)
* no numpy: the angle -> pulse-width curve is a small pure-Python cubic least-squares fit
* no tqdm/readchar/pprint
* drawings can be streamed from a ``.lines`` text file (one polyline per line,
  ``x,y x,y x,y ...``) so that large drawings never have to fit in RAM
* a plot can be interrupted from another thread/task with ``request_stop()``

The module also runs under CPython (or MicroPython's unix port), where it falls back to virtual
mode, which is handy for testing.
"""

import json
import math
import sys

try:
    from time import ticks_us, ticks_diff, sleep_ms
except ImportError:  # CPython
    import time as _time

    def ticks_us():
        return int(_time.monotonic() * 1000000)

    def ticks_diff(a, b):
        return a - b

    def sleep_ms(ms):
        _time.sleep(ms / 1000)


try:
    from machine import Pin, PWM
except ImportError:  # unix port / CPython: no hardware, use virtual mode
    Pin = PWM = None


def sleep_s(seconds):
    if seconds > 0:
        sleep_ms(int(seconds * 1000))


class PlotStopped(Exception):
    pass


# ----------------- small numeric helpers (numpy replacements) -----------------


class Polynomial:
    """A least-squares polynomial fit of ``ys`` against ``xs`` (like numpy.poly1d(numpy.polyfit())).

    The x values are centred and scaled to [-1, 1] before fitting, which keeps the normal
    equations well conditioned even with the single-precision floats of the ESP32.
    """

    def __init__(self, xs, ys, degree=3):
        n = len(xs)
        degree = min(degree, n - 1)
        m = degree + 1
        self.mid = sum(xs) / n
        self.scale = max(abs(x - self.mid) for x in xs) or 1.0

        # build the normal equations A.c = b as an augmented matrix
        a = [[0.0] * (m + 1) for _ in range(m)]
        for x, y in zip(xs, ys):
            u = (x - self.mid) / self.scale
            powers = [u**k for k in range(m)]
            for i in range(m):
                row = a[i]
                for j in range(m):
                    row[j] += powers[i] * powers[j]
                row[m] += powers[i] * y

        # Gaussian elimination with partial pivoting
        for col in range(m):
            pivot = max(range(col, m), key=lambda r: abs(a[r][col]))
            a[col], a[pivot] = a[pivot], a[col]
            p = a[col][col]
            if p == 0:
                raise ValueError("cannot fit polynomial to these values")
            for r in range(col + 1, m):
                f = a[r][col] / p
                if f:
                    for c in range(col, m + 1):
                        a[r][c] -= f * a[col][c]

        coeffs = [0.0] * m
        for i in range(m - 1, -1, -1):
            s = a[i][m]
            for j in range(i + 1, m):
                s -= a[i][j] * coeffs[j]
            coeffs[i] = s / a[i][i]

        self.coeffs = coeffs  # lowest power first, in scaled units

    def __call__(self, x):
        u = (x - self.mid) / self.scale
        result = 0.0
        for c in reversed(self.coeffs):
            result = result * u + c
        return result


def _mean(values):
    return sum(values) / len(values)


# ----------------- reading line data -----------------


def iter_points(text):
    """Yield (x, y) from a ``"x,y x,y ..."`` string without building big intermediate lists."""
    i = 0
    n = len(text)
    while i < n:
        while i < n and text[i] in " \t\r\n":
            i += 1
        if i >= n:
            return
        j = i
        while j < n and text[j] not in " \t\r\n":
            j += 1
        token = text[i:j]
        c = token.find(",")
        if c > 0:
            yield float(token[:c]), float(token[c + 1 :])
        i = j + 1


class LinesFile:
    """Re-iterable source of lines from a ``.lines`` file; each line is an iterator of points."""

    def __init__(self, filename):
        self.filename = filename

    def __iter__(self):
        with open(self.filename, "r") as f:
            while True:
                text = f.readline()
                if not text:
                    break
                text = text.strip()
                if not text or text[0] == "#":
                    continue
                yield iter_points(text)


class LinesList:
    """Re-iterable source of lines from an in-memory list such as ``[[[x, y], [x, y]], ...]``."""

    def __init__(self, lines):
        self.lines = lines

    def __iter__(self):
        for line in self.lines:
            yield ((p[0], p[1]) for p in line)


class Plotter:
    def __init__(
        self,
        virtual=False,  # a virtual plotter runs in software only
        turtle=False,  # kept for signature compatibility; not available on MicroPython
        turtle_coarseness=None,
        #  ----------------- geometry of the plotter -----------------
        bounds=(-10, 5, 10, 15),  # the maximum rectangular drawing area
        #  ----------------- naive calculation values -----------------
        servo_1_parked_pw=1500,  # pulse-widths when parked
        servo_2_parked_pw=1500,
        servo_1_degree_ms=-10,  # microseconds pulse-width per degree
        servo_2_degree_ms=10,
        servo_1_parked_angle=0,  # the arm angle in the parked position
        servo_2_parked_angle=0,
        #  ----------------- hysteresis -----------------
        hysteresis_correction_1=0,  # hardware error compensation
        hysteresis_correction_2=0,
        #  ----------------- servo angles and pulse-widths in lists -----------------
        servo_1_angle_pws=(),  # pulse-widths for various angles
        servo_2_angle_pws=(),
        #  ----------------- servo angles and pulse-widths in lists (bi-directional) ------
        servo_1_angle_pws_bidi=(),  # bi-directional pulse-widths for various angles
        servo_2_angle_pws_bidi=(),
        #  ----------------- the pen -----------------
        pw_up=None,  # pulse-widths for pen up/down
        pw_down=None,
        #  ----------------- physical control -----------------
        angular_step=None,  # default step of the servos in degrees
        wait=None,  # default wait time between operations
        resolution=None,  # default resolution of the plotter in cm
        #  ----------------- ESP32 GPIO pins -----------------
        servo_1_pin=25,  # shoulder
        servo_2_pin=26,  # elbow
        pen_pin=27,
        pwm_frequency=50,
    ):

        self.last_moved = ticks_us()
        self.virtual = virtual
        self.angle_1 = servo_1_parked_angle
        self.angle_2 = servo_2_parked_angle
        self.turtle = False
        if turtle:
            print("Turtle mode unavailable")

        self.bounds = bounds

        # stop/progress state, used by the web interface
        self._stop = False
        self.busy = False
        self.progress_done = 0
        self.progress_total = 0

        # If pulse-widths to angles are supplied for each servo, fit a cubic polynomial to them
        # (the Pi version uses numpy.polyfit for this). Otherwise, use a simple approximation based
        # on a centre of travel of 1500µS and 10µS per degree.

        self.servo_1_parked_pw = servo_1_parked_pw
        self.servo_1_degree_ms = servo_1_degree_ms
        self.servo_1_parked_angle = servo_1_parked_angle
        self.hysteresis_correction_1 = hysteresis_correction_1

        if servo_1_angle_pws_bidi:
            servo_1_angle_pws, self.hysteresis_correction_1 = self._from_bidi(
                servo_1_angle_pws_bidi
            )

        if servo_1_angle_pws:
            self.angles_to_pw_1 = Polynomial(
                [p[0] for p in servo_1_angle_pws], [p[1] for p in servo_1_angle_pws], 3
            )
        else:
            self.angles_to_pw_1 = self.naive_angles_to_pulse_widths_1

        self.servo_2_parked_pw = servo_2_parked_pw
        self.servo_2_degree_ms = servo_2_degree_ms
        self.servo_2_parked_angle = servo_2_parked_angle
        self.hysteresis_correction_2 = hysteresis_correction_2

        if servo_2_angle_pws_bidi:
            servo_2_angle_pws, self.hysteresis_correction_2 = self._from_bidi(
                servo_2_angle_pws_bidi
            )

        if servo_2_angle_pws:
            self.angles_to_pw_2 = Polynomial(
                [p[0] for p in servo_2_angle_pws], [p[1] for p in servo_2_angle_pws], 3
            )
        else:
            self.angles_to_pw_2 = self.naive_angles_to_pulse_widths_2

        # set some initial values required for moving methods
        self.previous_pw_1 = self.previous_pw_2 = 0
        self.active_hysteresis_correction_1 = self.active_hysteresis_correction_2 = 0
        self.reset_report()

        self.servo_1 = self.servo_2 = None

        if not self.virtual and PWM is None:
            print("machine.PWM is not available; running in virtual mode")
            self.virtual = True

        if self.virtual:
            self.wait = wait or 0
            self.virtualise()

        else:
            # 50Hz: the pulse frequency should be no higher than 100Hz
            self.servo_1 = Servo(servo_1_pin, pwm_frequency)
            self.servo_2 = Servo(servo_2_pin, pwm_frequency)
            # by default we use a wait factor of 0.01 seconds for better control
            self.wait = wait if wait is not None else 0.01

        # create the pen object
        pw_up = pw_up or 1400
        pw_down = pw_down or 1600

        self.pen = Pen(
            bg=self,
            pw_up=pw_up,
            pw_down=pw_down,
            pin=pen_pin,
            virtual=self.virtual,
            frequency=pwm_frequency,
        )

        self.angular_step = angular_step or 0.1
        self.resolution = resolution or 0.1

        self.set_angles(self.servo_1_parked_angle, self.servo_2_parked_angle)
        sleep_ms(1000)

        self.status()

    @staticmethod
    def _from_bidi(angle_pws_bidi):
        """Use bi-directional values to obtain mean values and a hysteresis correction value."""
        angle_pws = []
        differences = []
        for angle, pws in angle_pws_bidi.items():
            angle_pws.append([angle, (pws["acw"] + pws["cw"]) / 2])
            differences.append((pws["acw"] - pws["cw"]) / 2)
        return angle_pws, _mean(differences)

    def virtualise(self):

        print("Initialising virtual BrachioGraph")

        self.virtual_pw_1 = self.angles_to_pw_1(-90)
        self.virtual_pw_2 = self.angles_to_pw_2(90)
        self.virtual = True

    #  ----------------- stop / progress -----------------

    def request_stop(self):
        """Ask a running plot to stop at the next step (safe to call from another thread)."""
        self._stop = True

    def _check_stop(self):
        if self._stop:
            self._stop = False
            raise PlotStopped()

    def _progress(self, done, total):
        previous = self.progress_done
        self.progress_done, self.progress_total = done, total
        if total and (done * 10 // total) != (previous * 10 // total):
            print("Lines: {}/{} ({}%)".format(done, total, done * 100 // total))

    #  ----------------- plotting methods -----------------

    def plot_file(self, filename="", bounds=None, angular_step=None, wait=None, resolution=None):
        """Plots an image encoded as lines in ``filename``.

        * ``.lines`` files (one polyline per text line: ``x,y x,y ...``) are streamed from flash,
          so they can be as large as the filesystem allows.
        * ``.json`` files (as produced by linedraw) are loaded into memory - fine for small ones.
        """

        bounds = bounds or self.bounds

        if filename.endswith(".json"):
            with open(filename, "r") as line_file:
                source = LinesList(json.load(line_file))
        else:
            source = LinesFile(filename)

        self._plot_source(source, bounds, angular_step, wait, resolution, flip=True)

    def plot_lines(
        self,
        lines=[],
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        flip=False,
        rotate=False,
    ):
        """Draws each line in ``lines``. As in the original, the image is always flipped."""

        bounds = bounds or self.bounds
        self._plot_source(LinesList(lines), bounds, angular_step, wait, resolution, flip=True)

    def _plot_source(self, source, bounds, angular_step, wait, resolution, flip=True):
        # first pass: find the extent of the drawing
        min_x = min_y = 1e30
        max_x = max_y = -1e30
        total = 0
        for line in source:
            total += 1
            for x, y in line:
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y

        if not total:
            print("Nothing to plot")
            return

        transform = self._transformer(min_x, max_x, min_y, max_y, bounds, flip)

        # second pass: draw
        self._stop = False
        self._progress(0, total)
        done = 0
        try:
            for line in source:
                first = True
                for point in line:
                    x, y = transform(point[0], point[1])
                    if first:
                        first = False
                        # only if we are not within 1mm of the start of the line, lift pen and go
                        if (round(self.x, 1), round(self.y, 1)) != (round(x, 1), round(y, 1)):
                            self.xy(x, y, angular_step, wait, resolution)
                    else:
                        self.xy(x, y, angular_step, wait, resolution, draw=True)
                done += 1
                self._progress(done, total)
        finally:
            self.park()

    #  ----------------- pattern-drawing methods -----------------

    def box(
        self, bounds=None, angular_step=None, wait=None, resolution=None, repeat=1, reverse=False
    ):
        """Draw a box marked out by the ``bounds``."""

        bounds = bounds or self.bounds

        if not bounds:
            return "Box drawing is only possible when the bounds attribute is set."

        self._stop = False
        self.xy(bounds[0], bounds[1], angular_step, wait, resolution)

        for r in range(repeat):

            if not reverse:
                corners = ((2, 1), (2, 3), (0, 3), (0, 1))
            else:
                corners = ((0, 3), (2, 3), (2, 1), (0, 1))

            for cx, cy in corners:
                self.xy(bounds[cx], bounds[cy], angular_step, wait, resolution, draw=True)

        self.park()

    def test_pattern(
        self,
        lines=4,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        repeat=1,
        reverse=False,
        both=False,
    ):

        self._stop = False
        self.vertical_lines(lines, bounds, angular_step, wait, resolution, repeat, reverse, both)
        self.horizontal_lines(lines, bounds, angular_step, wait, resolution, repeat, reverse, both)

    def vertical_lines(
        self,
        lines=4,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        repeat=1,
        reverse=False,
        both=False,
    ):

        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when the bounds attribute is set."

        if not reverse:
            top_y = self.top
            bottom_y = self.bottom
        else:
            bottom_y = self.top
            top_y = self.bottom

        for n in range(repeat):
            step = (self.right - self.left) / lines
            x = self.left
            while x <= self.right:
                self.draw_line((x, top_y), (x, bottom_y), angular_step, wait, resolution, both)
                x = x + step

        self.park()

    def horizontal_lines(
        self,
        lines=4,
        bounds=None,
        angular_step=None,
        wait=None,
        resolution=None,
        repeat=1,
        reverse=False,
        both=False,
    ):

        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when the bounds attribute is set."

        if not reverse:
            min_x = self.left
            max_x = self.right
        else:
            max_x = self.left
            min_x = self.right

        for n in range(repeat):
            step = (self.bottom - self.top) / lines
            y = self.top
            while y >= self.bottom:
                self.draw_line((min_x, y), (max_x, y), angular_step, wait, resolution, both)
                y = y + step

        self.park()

    #  ----------------- x/y drawing methods -----------------

    def draw_line(
        self, start=(0, 0), end=(0, 0), angular_step=None, wait=None, resolution=None, both=False
    ):
        """Draws a line between two points"""

        start_x, start_y = start
        end_x, end_y = end

        self.xy(start_x, start_y, angular_step, wait, resolution)

        self.xy(end_x, end_y, angular_step, wait, resolution, draw=True)

        if both:
            self.xy(start_x, start_y, angular_step, wait, resolution, draw=True)

    def xy(self, x=None, y=None, angular_step=None, wait=None, resolution=None, draw=False):
        """Moves the pen to the xy position; optionally draws while doing it. ``None`` for x or y
        means that the pen will not be moved in that dimension.
        """

        wait = wait if wait is not None else self.wait
        resolution = resolution or self.resolution

        x = x if x is not None else self.x
        y = y if y is not None else self.y
        (angle_1, angle_2) = self.xy_to_angles(x, y)

        if draw:

            # calculate how many steps we need for this move, and the x/y length of each
            (x_length, y_length) = (x - self.x, y - self.y)

            length = math.sqrt(x_length**2 + y_length**2)

            no_of_steps = round(length / resolution) or 1

            (length_of_step_x, length_of_step_y) = (x_length / no_of_steps, y_length / no_of_steps)

            for step in range(no_of_steps):

                self.x = self.x + length_of_step_x
                self.y = self.y + length_of_step_y

                angle_1, angle_2 = self.xy_to_angles(self.x, self.y)
                self.move_angles(angle_1, angle_2, angular_step, wait, draw)

        else:
            self.move_angles(angle_1, angle_2, angular_step, wait, draw)

    #  ----------------- servo angle drawing methods -----------------

    def move_angles(self, angle_1=None, angle_2=None, angular_step=None, wait=None, draw=False):
        """Moves the servo motors to the specified angles step-by-step, calling ``set_angles()`` for
        each step. ``None`` for one of the angles means that that servo will not move.
        """

        wait = wait if wait is not None else self.wait
        angular_step = angular_step or self.angular_step
        wait_us = int(wait * 1000000)

        if draw:
            self.pen.down()
        else:
            self.pen.up()

        diff_1 = diff_2 = 0

        if angle_1 is not None:
            diff_1 = angle_1 - self.angle_1
        if angle_2 is not None:
            diff_2 = angle_2 - self.angle_2

        no_of_steps = int(max(abs(diff_1 / angular_step), abs(diff_2 / angular_step))) or 1

        (length_of_step_1, length_of_step_2) = (diff_1 / no_of_steps, diff_2 / no_of_steps)

        for step in range(no_of_steps):

            self._check_stop()

            self.angle_1 = self.angle_1 + length_of_step_1
            self.angle_2 = self.angle_2 + length_of_step_2

            elapsed = ticks_diff(ticks_us(), self.last_moved)
            if elapsed < wait_us:
                sleep_ms((wait_us - elapsed) // 1000)

            self.set_angles(self.angle_1, self.angle_2)

            self.last_moved = ticks_us()

    # ----------------- pen-moving methods -----------------

    def set_angles(self, angle_1=None, angle_2=None):
        """Moves the servo motors to the specified angles immediately. Relies upon getting accurate
        pulse-width values. ``None`` for one of the angles means that that servo will not move.

        Calls ``set_pulse_widths()``.

        Sets ``current_x``, ``current_y``.
        """

        pw_1 = pw_2 = None

        if angle_1 is not None:
            pw_1 = self.angles_to_pw_1(angle_1)

            if pw_1 > self.previous_pw_1:
                self.active_hysteresis_correction_1 = self.hysteresis_correction_1
            elif pw_1 < self.previous_pw_1:
                self.active_hysteresis_correction_1 = -self.hysteresis_correction_1

            self.previous_pw_1 = pw_1

            pw_1 = pw_1 + self.active_hysteresis_correction_1

            self.angle_1 = angle_1
            self.angles_used_1.add(int(angle_1))
            self.pulse_widths_used_1.add(int(pw_1))

        if angle_2 is not None:
            pw_2 = self.angles_to_pw_2(angle_2)

            if pw_2 > self.previous_pw_2:
                self.active_hysteresis_correction_2 = self.hysteresis_correction_2
            elif pw_2 < self.previous_pw_2:
                self.active_hysteresis_correction_2 = -self.hysteresis_correction_2

            self.previous_pw_2 = pw_2

            pw_2 = pw_2 + self.active_hysteresis_correction_2

            self.angle_2 = angle_2
            self.angles_used_2.add(int(angle_2))
            self.pulse_widths_used_2.add(int(pw_2))

        self.x, self.y = self.angles_to_xy(self.angle_1, self.angle_2)

        self.set_pulse_widths(pw_1, pw_2)

    def park(self):
        """Park the plotter."""

        if self.virtual:
            print("Parking")

        self._stop = False
        self.pen.up()

        self.move_angles(self.servo_1_parked_angle, self.servo_2_parked_angle)

    #  ----------------- angles-to-pulse-widths methods -----------------

    def naive_angles_to_pulse_widths_1(self, angle):
        """A rule-of-thumb calculation of pulse-width for the desired servo angle"""
        return (angle - self.servo_1_parked_angle) * self.servo_1_degree_ms + self.servo_1_parked_pw

    def naive_angles_to_pulse_widths_2(self, angle):
        """A rule-of-thumb calculation of pulse-width for the desired servo angle"""
        return (angle - self.servo_2_parked_angle) * self.servo_2_degree_ms + self.servo_2_parked_pw

    # ----------------- line-processing methods -----------------

    def _transformer(self, min_x, max_x, min_y, max_y, bounds, flip):
        """Returns a function that maps image co-ordinates into the drawing ``bounds``, rotating
        by 90˚ if that gives a better fit (see ``analyse_lines()``)."""
        (
            rotate,
            x_mid_point,
            y_mid_point,
            box_x_mid_point,
            box_y_mid_point,
            divider,
        ) = self._analyse_extent(min_x, max_x, min_y, max_y, bounds)

        mirror = bool(flip) != bool(rotate)

        def transform(x, y):
            if rotate:
                x, y = y, x
            x = (x - x_mid_point) / divider  # centre on zero and scale to fit the box
            if mirror:  # flip before moving back into drawing pane
                x = -x
            x = x + box_x_mid_point
            y = (y - y_mid_point) / divider + box_y_mid_point
            return x, y

        return transform

    def rotate_and_scale_lines(self, lines=[], rotate=False, flip=False, bounds=None):
        """Rotates and scales the lines so that they best fit the available drawing ``bounds``."""
        bounds = bounds or self.bounds
        xs = [p[0] for line in lines for p in line]
        ys = [p[1] for line in lines for p in line]
        transform = self._transformer(min(xs), max(xs), min(ys), max(ys), bounds, flip)

        for line in lines:
            for point in line:
                point[0], point[1] = transform(point[0], point[1])

        return lines

    def analyse_lines(self, lines=[], rotate=False, bounds=None):
        """Analyses the co-ordinates in ``lines``, and returns:

        * ``rotate``: ``True`` if the image needs to be rotated by 90˚ in order to fit better
        * ``x_mid_point``, ``y_mid_point``: mid-points of the image
        * ``box_x_mid_point``, ``box_y_mid_point``: mid-points of the ``bounds``
        * ``divider``: the value by which we must divide all x and y so that they will fit safely
          inside the bounds.
        """
        xs = [p[0] for line in lines for p in line]
        ys = [p[1] for line in lines for p in line]
        return self._analyse_extent(min(xs), max(xs), min(ys), max(ys), bounds or self.bounds)

    def _analyse_extent(self, min_x, max_x, min_y, max_y, bounds):

        x_range, y_range = max_x - min_x, max_y - min_y
        box_x_range, box_y_range = bounds[2] - bounds[0], bounds[3] - bounds[1]

        x_mid_point, y_mid_point = (max_x + min_x) / 2, (max_y + min_y) / 2
        box_x_mid_point, box_y_mid_point = (bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2

        # If both image and box are in portrait orientation, or both in landscape, we don't need to
        # rotate the plot.

        if (x_range >= y_range and box_x_range >= box_y_range) or (
            x_range <= y_range and box_x_range <= box_y_range
        ):
            divider = max((x_range / box_x_range), (y_range / box_y_range))
            rotate = False

        else:
            divider = max((x_range / box_y_range), (y_range / box_x_range))
            rotate = True
            x_mid_point, y_mid_point = y_mid_point, x_mid_point

        return (rotate, x_mid_point, y_mid_point, box_x_mid_point, box_y_mid_point, divider or 1)

    # ----------------- physical control methods -----------------

    def set_pulse_widths(self, pw_1=None, pw_2=None):
        """Applies the supplied pulse-width values to the servos, or pretends to, if we're in
        virtual mode.
        """

        if self.virtual:

            if pw_1:
                if 500 < pw_1 < 2500:
                    self.virtual_pw_1 = int(pw_1)
                else:
                    raise ValueError("pulse-width {} out of range".format(pw_1))

            if pw_2:
                if 500 < pw_2 < 2500:
                    self.virtual_pw_2 = int(pw_2)
                else:
                    raise ValueError("pulse-width {} out of range".format(pw_2))

        else:

            if pw_1:
                self.servo_1.set_pw(pw_1)
            if pw_2:
                self.servo_2.set_pw(pw_2)

    def get_pulse_widths(self):
        """Returns the actual pulse-widths values; if in virtual mode, returns the nominal values -
        i.e. the values that they might be.
        """

        if self.virtual:
            return (self.virtual_pw_1, self.virtual_pw_2)

        return (self.servo_1.pw, self.servo_2.pw)

    def quiet(self, servos=None):
        """Stop sending pulses to the servos, so that they are no longer energised (and so that they
        stop buzzing).
        """

        if self.virtual:
            print("Going quiet")

        else:
            self.servo_1.set_pw(0)
            self.servo_2.set_pw(0)
            self.pen.pw(0)

    # ----------------- manual driving methods -----------------

    def capture_pws(self):
        """
        Helps capture angle/pulse-width data for the servos, as a dictionary to be used
        in a Plotter definition. Run it from the REPL (e.g. ``mpremote connect ... repl``).
        """

        print(
            """
Drive each servo over a wide range of movement (do not exceed a pulse-width
range ~600 to ~2400). To capture the pulse-width value for a particular angle,
press "c", then enter the angle. For each angle, do this in both directions,
clockwise and anti-clockwise. Press "0" to exit.
        """
        )

        pw_1, pw_2 = self.get_pulse_widths()
        pen_pw = self.pen.get_pw()

        last_action = values = None
        pws1_dict = {}
        pws2_dict = {}
        pen_pw_dict = {}

        print("0 to exit, c to capture a value, v to show captured values")
        print("Shoulder a: -10  A: -1   s: +10  S: +1")
        print("Elbow    k: -10  K: -1   l: +10  L: +1")
        print("Pen      z: -10          x: +10")

        controls = {
            "a": [-10, 0, 0, "acw"],
            "A": [-1, 0, 0, "acw"],
            "s": [+10, 0, 0, "cw"],
            "S": [+1, 0, 0, "cw"],
            "k": [0, -10, 0, "acw"],
            "K": [0, -1, 0, "acw"],
            "l": [0, +10, 0, "cw"],
            "L": [0, +1, 0, "cw"],
            "z": [0, 0, -10],
            "x": [0, 0, +10],
        }

        while True:
            key = readchar()
            values = controls.get(key)

            if values:

                if values[0] or values[1] or values[2]:
                    pw_1 += values[0]
                    pw_2 += values[1]
                    pen_pw += values[2]

                    print("shoulder: {}, elbow: {}, pen: {}".format(pw_1, pw_2, pen_pw))

                    self.set_pulse_widths(pw_1, pw_2)
                    self.pen.pw(pen_pw)

                    last_action = values

            elif key == "0" or key == "v":
                print("servo_1_angle_pws_bidi =")
                _pp(pws1_dict)
                print("servo_2_angle_pws_bidi =")
                _pp(pws2_dict)
                print("Pen pulse-widths =")
                _pp(pen_pw_dict)

                if key == "0":
                    return

            elif key == "c":
                if not last_action:
                    print("Drive the servos to a new position first")

                elif last_action[0]:
                    angle = int(input("Enter the angle of the inner arm: "))
                    pws1_dict.setdefault(angle, {})[last_action[3]] = pw_1
                    print(pws1_dict)

                elif last_action[1]:
                    angle = int(input("Enter the angle of the outer arm: "))
                    pws2_dict.setdefault(angle, {})[last_action[3]] = pw_2
                    print(pws2_dict)

                elif last_action[2]:
                    state = input("Enter the state of the pen ([u]p, [d]own):")
                    pen_pw_dict[state] = pen_pw
                    print(pen_pw)

    def drive_xy(self):
        """Control the x/y position using the keyboard."""

        while True:
            key = readchar()

            if key == "0":
                return
            elif key == "a":
                self.x = self.x - 1
            elif key == "s":
                self.x = self.x + 1
            elif key == "A":
                self.x = self.x - 0.1
            elif key == "S":
                self.x = self.x + 0.1
            elif key == "k":
                self.y = self.y - 1
            elif key == "l":
                self.y = self.y + 1
            elif key == "K":
                self.y = self.y - 0.1
            elif key == "L":
                self.y = self.y + 0.1

            print(self.x, self.y)

            self.xy(self.x, self.y)

    # ----------------- reporting methods -----------------

    def status(self):
        """Provides a report of the plotter status. Subclasses should override this to
        report on their own status."""

        line = "------------------------------------------"
        print(line)
        print("                      | Servo 1 | Servo 2 ")
        print("----------------------|---------|---------")

        pw_1, pw_2 = self.get_pulse_widths()
        print("{:>23} {:>7.0f} | {:>7.0f}".format("pulse-width |", pw_1, pw_2))

        angle_1, angle_2 = self.angle_1, self.angle_2
        print("{:>23} {:>7.0f} | {:>7.0f}".format("angle |", angle_1, angle_2))

        h1, h2 = self.hysteresis_correction_1, self.hysteresis_correction_2
        print("{:>23} {:>7.1f} | {:>7.1f}".format("hysteresis correction |", h1, h2))
        print(line)
        print("{:>23} {:>7.1f} | {:>7.1f}".format("x/y location |", self.x, self.y))
        print()
        print(line)
        print("pen:", self.pen.position)

        print(line)
        print(
            "left: {}, right: {}, top: {}, bottom: {}".format(
                self.left, self.right, self.top, self.bottom
            )
        )
        print(line)
        print("wait: {} seconds".format(self.wait))
        print(line)
        print("resolution: {} cm".format(self.resolution))
        print(line)
        print("angular step: {}˚".format(self.angular_step))
        print(line)

    @property
    def left(self):
        return self.bounds[0]

    @property
    def bottom(self):
        return self.bounds[1]

    @property
    def right(self):
        return self.bounds[2]

    @property
    def top(self):
        return self.bounds[3]

    def reset_report(self):

        self.angle_1 = self.angle_2 = None

        # Record the range of movement of the plotter (the Pi version keeps sets of every value,
        # which grow into large allocations; only min/max are ever reported).
        self.angles_used_1 = Range()
        self.angles_used_2 = Range()
        self.pulse_widths_used_1 = Range()
        self.pulse_widths_used_2 = Range()

    # ----------------- trigonometric methods -----------------

    def xy_to_angles(self, x=0, y=0):
        """Return the servo angles required to reach any x/y position. This is a dummy method in
        the base class; it needs to be overridden in a sub-class implementation."""
        return (0, 0)

    def angles_to_xy(self, angle_1, angle_2):
        """Return the servo angles required to reach any x/y position. This is a dummy method in
        the base class; it needs to be overridden in a sub-class implementation."""
        return (0, 0)


class Range:
    """Tracks the minimum and maximum of the values added; ``min()``/``max()`` work on it."""

    def __init__(self):
        self.lo = self.hi = None

    def add(self, v):
        if self.lo is None or v < self.lo:
            self.lo = v
        if self.hi is None or v > self.hi:
            self.hi = v

    def __len__(self):
        return 0 if self.lo is None else 2

    def __iter__(self):
        if self.lo is not None:
            yield self.lo
            yield self.hi


class Servo:
    """A hobby servo on an ESP32 GPIO pin, driven by ``machine.PWM``."""

    def __init__(self, pin, frequency=50):
        self.pin = pin
        self.pw = 0
        self.pwm = PWM(Pin(pin), freq=frequency, duty_ns=0)

    def set_pw(self, pw):
        """Set the pulse-width in µs; 0 stops the pulses (like pigpio's set_servo_pulsewidth)."""
        pw = int(pw)
        if pw and not 500 <= pw <= 2500:
            raise ValueError("pulse-width {} out of range on GPIO {}".format(pw, self.pin))
        self.pwm.duty_ns(pw * 1000)
        self.pw = pw

    def deinit(self):
        self.pwm.deinit()


class Pen:
    def __init__(
        self,
        bg,
        pw_up=1700,
        pw_down=1300,
        pin=27,
        transition_time=0.25,
        virtual=False,
        frequency=50,
    ):

        self.bg = bg
        self.pin = pin
        self.pw_up = pw_up
        self.pw_down = pw_down
        self.transition_time = transition_time
        self.position = "down"
        self.virtual = virtual
        self.virtual_pw = pw_down

        if self.virtual:
            print("Initialising virtual Pen")
            self.servo = None
        else:
            self.servo = Servo(pin, frequency)

        self.up()

    def down(self):

        if self.position == "up":

            if self.virtual:
                self.virtual_pw = self.pw_down
            else:
                self.ease_pen(self.pw_up, self.pw_down)

            self.position = "down"

    def up(self):

        if self.position == "down":

            if self.virtual:
                self.virtual_pw = self.pw_up
            else:
                self.ease_pen(self.pw_down, self.pw_up)

            self.position = "up"

    def ease_pen(self, start, end):
        """
        Moves the pen gently instead of all at once. Slower but reduces marking on the paper.
        """
        diff = int(end - start)
        if not diff:
            self.servo.set_pw(end)
            return
        pw = start
        length_of_step = 1 if diff > 0 else -1

        for i in range(abs(diff)):
            pw += length_of_step
            self.servo.set_pw(pw)
            sleep_ms(1)

    # for convenience, a quick way to set pen motor pulse-widths
    def pw(self, pulse_width):

        if self.virtual:
            self.virtual_pw = pulse_width
        else:
            self.servo.set_pw(pulse_width)

    # for convenience, a quick way to get pen motor pulse-widths
    def get_pw(self):

        if self.virtual:
            return self.virtual_pw
        return self.servo.pw


# ----------------- REPL helpers -----------------


def readchar():
    """Read a single keypress (MicroPython REPL delivers characters unbuffered)."""
    if sys.implementation.name == "micropython":
        return sys.stdin.read(1)
    try:
        import readchar as _rc  # CPython fallback

        return _rc.readchar()
    except ImportError:
        return input()[:1]


def _pp(d):
    print("{")
    for k in sorted(d):
        print("    {!r}: {!r},".format(k, d[k]))
    print("}")
