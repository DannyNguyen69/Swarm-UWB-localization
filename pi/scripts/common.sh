#!/bin/bash
# Sourced by the other scripts: project root, config, Python and fc_router locations.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set -a
. "$ROOT/config/uwb.env"
set +a
PY="$ROOT/uwb/.venv/bin/python"
[ -x "$PY" ] || PY=python3
ROUTER="${ROUTER:-$ROOT/build/fc_router}"
LOGS="$ROOT/logs"
mkdir -p "$LOGS"
