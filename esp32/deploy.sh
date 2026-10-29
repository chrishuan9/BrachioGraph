#!/usr/bin/env bash
# Copy the BrachioGraph ESP32 port to the board with mpremote (pip install mpremote).
#
#   ./deploy.sh [serial-port]      e.g. ./deploy.sh /dev/cu.usbserial-5B340326631
#
# Copies the code, gzipped web interface, and a few drawings converted from ../images and
# ../test-patterns, then resets the board. Existing drawings on the board are kept.
set -euo pipefail
cd "$(dirname "$0")"

PORT="${1:-/dev/cu.usbserial-5B340326631}"
MP=(mpremote connect "$PORT")
PY="${PYTHON:-python3}"

rm -rf build && mkdir -p build/www build/drawings

# gzip the web interface (smaller on flash and much faster over Wi-Fi)
for f in www/*; do gzip -9 -c "$f" > "build/www/$(basename "$f").gz"; done

# starter drawings
"$PY" tools/json2lines.py ../images/switzerland.json build/drawings/switzerland.lines
for f in ../test-patterns/*.json; do
  "$PY" tools/json2lines.py "$f" "build/drawings/$(basename "${f%.json}").lines"
done

FILES=(plotter.py brachiograph.py webserver.py config.py bg.py main.py)
[ -f wifi_secrets.py ] && FILES+=(wifi_secrets.py)

"${MP[@]}" mkdir :www >/dev/null 2>&1 || true
"${MP[@]}" mkdir :drawings >/dev/null 2>&1 || true
# remove an uncompressed copy from an earlier deploy so the .gz one is served
"${MP[@]}" rm :www/index.html >/dev/null 2>&1 || true
"${MP[@]}" rm :www/linedraw.js >/dev/null 2>&1 || true

"${MP[@]}" cp "${FILES[@]}" : \
  + cp build/www/index.html.gz build/www/linedraw.js.gz :www/ \
  + cp build/drawings/*.lines :drawings/ \
  + reset

echo
echo "Done. Watch the boot messages (and the IP address) with:"
echo "  mpremote connect $PORT repl"
