#!/bin/bash
# One-time setup on the Pi: Python venv with the packages uwb_bridge needs (needs internet).
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
python3 -m venv "$ROOT/uwb/.venv"
"$ROOT/uwb/.venv/bin/pip" install --upgrade pip
"$ROOT/uwb/.venv/bin/pip" install pymavlink pyserial numpy
"$ROOT/uwb/.venv/bin/python" -c "import pymavlink, serial, numpy; print('OK: pymavlink', pymavlink.__version__, '| pyserial', serial.VERSION, '| numpy', numpy.__version__)"
