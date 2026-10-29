#!/usr/bin/env python3
"""Convert linedraw/BrachioGraph JSON line files into the streamable ``.lines`` format.

    python3 tools/json2lines.py ../images/switzerland.json drawings/switzerland.lines

``.lines`` format: one polyline per text line, points as ``x,y`` separated by spaces. Long
polylines are split into chunks (the next chunk starts on the last point of the previous one,
so the pen stays down) to keep the memory needed per line small on the ESP32.
"""

import json
import sys

MAX_POINTS = 200


def fmt(v):
    v = round(float(v), 2)
    return str(int(v)) if v == int(v) else str(v)


def convert(lines, out, max_points=MAX_POINTS):
    count = 0
    for line in lines:
        if len(line) < 1:
            continue
        start = 0
        while True:
            chunk = line[start : start + max_points]
            out.write(" ".join(fmt(p[0]) + "," + fmt(p[1]) for p in chunk) + "\n")
            count += 1
            if start + max_points >= len(line):
                break
            start += max_points - 1  # overlap by one point so the line is continuous
    return count


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1]) as f:
        lines = json.load(f)
    with open(sys.argv[2], "w") as out:
        n = convert(lines, out)
    print("wrote {} lines to {}".format(n, sys.argv[2]))


if __name__ == "__main__":
    main()
