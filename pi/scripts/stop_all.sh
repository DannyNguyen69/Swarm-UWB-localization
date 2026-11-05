#!/bin/bash
# Stops what run_all.sh started.
. "$(dirname "$0")/common.sh"
for n in uwb router; do
  if [ -f "$LOGS/$n.pid" ]; then
    kill "$(cat "$LOGS/$n.pid")" 2>/dev/null && echo "đã dừng $n"
    rm -f "$LOGS/$n.pid"
  fi
done
