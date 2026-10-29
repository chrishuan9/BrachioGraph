# Runs at boot: join Wi-Fi (or start an access point), create the plotter, start the web server.
#
# Hold the BOOT button (GPIO 0) while resetting to skip the web server and stay at the REPL.
# From the REPL, press Ctrl-C to stop the server; then `from bg import bg` gives you the plotter.

import time
import config


def connect_wifi():
    import network

    try:
        network.hostname(config.HOSTNAME)
    except Exception:
        pass

    sta = network.WLAN(network.STA_IF)
    sta.active(True)

    if config.WIFI_SSID:
        if not sta.isconnected():
            print("Connecting to Wi-Fi", config.WIFI_SSID, end="")
            sta.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
            for _ in range(40):
                if sta.isconnected():
                    break
                print(".", end="")
                time.sleep(0.5)
            print()
        if sta.isconnected():
            ip = sta.ifconfig()[0]
            print("Connected: http://{}/  (or http://{}.local/)".format(ip, config.HOSTNAME))
            return ip
        print("Could not join", config.WIFI_SSID, "- starting access point instead")

    sta.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=config.AP_SSID, password=config.AP_PASSWORD, authmode=3)  # WPA2
    ip = ap.ifconfig()[0]
    print("Access point '{}' (password '{}'): http://{}/".format(config.AP_SSID, config.AP_PASSWORD, ip))
    return ip


def boot_button_held():
    try:
        from machine import Pin

        return Pin(0, Pin.IN, Pin.PULL_UP).value() == 0
    except Exception:
        return False


if boot_button_held():
    print("BOOT button held: web server not started")
else:
    connect_wifi()
    from bg import bg
    import webserver

    webserver.run(bg, port=config.HTTP_PORT)
