#!/bin/bash
# Tells the tag to re-measure the anchor-to-anchor distances (only with the drone on the ground).
# Works while uwb_bridge is running. The bridge picks up the new layout while disarmed.
. "$(dirname "$0")/common.sh"
exec "$PY" -u "$ROOT/uwb/uwb_bridge.py" --resurvey "$@"
