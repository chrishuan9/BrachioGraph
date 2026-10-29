# BrachioGraph on ESP32 (MicroPython)

A MicroPython port of the BrachioGraph plotter code for a plain ESP32 (4 MB flash, no PSRAM),
with a web interface for uploading images and plotting them.

* **No pigpio.** The three servos are driven by `machine.PWM` at 50 Hz (`duty_ns`).
* **No numpy, tqdm, readchar.** The angle → pulse-width calibration curve is a small pure-Python
  cubic least-squares fit, which gives the same pulse-widths as `numpy.polyfit` on the Pi.
* **Same API.** `bg.box()`, `bg.test_pattern()`, `bg.plot_file()`, `bg.park()`, `bg.status()`,
  `bg.report()`, `bg.capture_pws()`, `bg.drive_xy()`, `bg.quiet()` all work from the REPL.
* **Web interface.** Upload JPG/PNG/SVG; the browser converts them to lines with a JavaScript port
  of `linedraw.py` (contours + hatching, same parameters as on the Pi), or traces the paths of an
  SVG directly. The result is previewed, saved to the ESP32's flash and plotted. Plotting
  runs in a background thread, so you can watch progress and stop it from the page.

## Files

| file | what it is |
|---|---|
| `plotter.py`, `brachiograph.py` | the ported plotter classes |
| `config.py` | Wi-Fi settings, servo calibration (copied from the Pi's `bg.py`), GPIO pins |
| `bg.py` | `from bg import bg` gives you the plotter, as on the Pi |
| `main.py` | runs at boot: joins Wi-Fi, creates the plotter, starts the web server |
| `webserver.py` | asyncio HTTP server + JSON API |
| `www/index.html`, `www/linedraw.js` | the web interface and the JS linedraw port |
| `tools/json2lines.py` | converts linedraw `.json` files to the `.lines` format |
| `deploy.sh` | copies everything to the board with `mpremote` |

## Wiring

Power the servos from a separate 5 V supply (not the ESP32's 3.3 V pin) and connect its ground to
the ESP32's GND. The 3.3 V signal from the ESP32 drives normal hobby servos (SG90/MG90) fine.

| servo | Pi GPIO (before) | ESP32 GPIO (default) |
|---|---|---|
| shoulder (servo 1) | 14 | **25** |
| elbow (servo 2) | 15 | **26** |
| pen lift | 18 | **27** |

The pins are set in `config.py`. Avoid strapping pins (0, 2, 5, 12, 15) and input-only pins (34–39).

## Install

You have already flashed MicroPython. From this folder, on your Mac:

```sh
source ../../.venv/bin/activate       # or any Python environment
pip install mpremote

cp wifi_secrets.example.py wifi_secrets.py   # then put your Wi-Fi name/password in it
./deploy.sh /dev/cu.usbserial-5B340326631
mpremote connect /dev/cu.usbserial-5B340326631 repl   # shows the IP address; Ctrl-] to leave
```

Then open `http://brachiograph.local/` (or the IP address printed at boot). If Wi-Fi isn't set
or can't be joined, the ESP32 starts its own network **BrachioGraph** (password
`brachiograph`); join it and open `http://192.168.4.1/`.

`deploy.sh` also puts `switzerland.lines` and the test patterns on the board.

## Using it

**From the browser:** drop an image on the page. For JPG/PNG the options are linedraw's:
*resolution* (1024), *draw_contours* (2), *draw_hatch* (16) and the repeat counts. For an SVG you
can trace its paths exactly or rasterise it through linedraw. Preview it, then **Save & plot**.
Existing linedraw `.json` files can be dropped in too.

**From the REPL** (`mpremote connect … repl`, press Ctrl-C to stop the web server):

```python
>>> from bg import bg
>>> bg.box()
>>> bg.test_pattern()
>>> bg.plot_file("drawings/switzerland.lines")
>>> bg.capture_pws()      # recalibrate the servos, as on the Pi
```

Hold the **BOOT** button while pressing **EN/reset** to boot straight to the REPL without
starting the web server.

## The `.lines` format

The ESP32 has about 150 kB of RAM, so drawings are stored as text with one polyline per line,
`x,y x,y x,y …`, and streamed from flash while plotting (one pass to find the size, one to draw).
Long polylines are split into chunks of up to 200 points; each chunk starts on the last point of
the previous one, so the pen stays down. `plot_file()` also accepts small `.json` files.

```sh
python3 tools/json2lines.py ../images/prague.json prague.lines
mpremote cp prague.lines :drawings/
```

## Differences from the Pi version

* Turtle graphics simulation isn't available.
* `angles_used_*` / `pulse_widths_used_*` only keep the min and max (that's all `report()` uses);
  keeping every value used too much RAM.
* `bg.request_stop()` stops a running plot at the next step; the web interface's Stop button
  uses it.
* The ESP32 uses single-precision floats, so pulse-widths can differ from the Pi's by a
  fraction of a microsecond.
