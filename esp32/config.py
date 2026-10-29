# BrachioGraph-on-ESP32 settings. Edit this file, then copy it to the board.

# ----------------- Wi-Fi -----------------
# Join your network if WIFI_SSID is set. If that fails (or WIFI_SSID is empty), the ESP32 starts
# its own access point instead: join AP_SSID with AP_PASSWORD and open http://192.168.4.1/
WIFI_SSID = ""
WIFI_PASSWORD = ""
HOSTNAME = "brachiograph"  # -> http://brachiograph.local/ on most networks
AP_SSID = "BrachioGraph"
AP_PASSWORD = "brachiograph"  # at least 8 characters
HTTP_PORT = 80

# Keep real credentials out of git: put WIFI_SSID / WIFI_PASSWORD in wifi_secrets.py
# (ignored by git, copied to the board by deploy.sh). See wifi_secrets.example.py.
try:
    from wifi_secrets import *  # noqa
except ImportError:
    pass

# ----------------- servo calibration -----------------
# Copied from the Pi's bg.py. Angles in degrees and corresponding pulse-widths (µs).

servo_1_angle_pws = [
    [-162, 2470],
    [-144, 2250],
    [-126, 2050],
    [-108, 1860],
    [-90, 1690],
    [-72, 1530],
    [-54, 1350],
    [-36, 1190],
    [-18, 1010],
    [0, 840],
    [18, 640],
]

servo_2_angle_pws = [
    [0, 660],
    [18, 840],
    [36, 1030],
    [54, 1180],
    [72, 1340],
    [90, 1490],
    [108, 1640],
    [126, 1830],
    [144, 2000],
    [162, 2200],
    [180, 2410],
]

PLOTTER = dict(
    # the lengths of the arms
    inner_arm=8,
    outer_arm=8,
    # the drawing area
    bounds=(-8, 4, 8, 13),
    # angles in degrees and corresponding pulse-widths for the two arm servos
    servo_1_angle_pws=servo_1_angle_pws,
    servo_2_angle_pws=servo_2_angle_pws,
    # pulse-widths for pen up/down
    pw_down=1200,
    pw_up=1850,
    # ESP32 GPIOs for the servo signal wires (the Pi used GPIO 14, 15 and 18).
    # Avoid strapping pins (0, 2, 5, 12, 15) and input-only pins (34-39).
    servo_1_pin=25,  # shoulder
    servo_2_pin=26,  # elbow
    pen_pin=27,  # pen lift
)
