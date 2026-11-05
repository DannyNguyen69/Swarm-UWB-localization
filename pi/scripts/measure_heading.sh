#!/bin/bash
# Reads the FC's yaw while the drone lies still with its nose along A1 -> A2, prints the HEADING value.
# Needs fc_router running (scripts/run_router.sh or run_all.sh). Nose along another line: --nose A:B (e.g. --nose 2:1, --nose 2:3).
. "$(dirname "$0")/common.sh"
exec "$PY" -u "$ROOT/uwb/uwb_bridge.py" --fw ardupilot --fc "udpout:127.0.0.1:$LOCAL_UDP" --measure-heading "$@"
