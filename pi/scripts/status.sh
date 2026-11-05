#!/bin/bash
# State of the background processes and the last lines of their logs.
. "$(dirname "$0")/common.sh"
for n in router uwb; do
  if [ -f "$LOGS/$n.pid" ] && kill -0 "$(cat "$LOGS/$n.pid")" 2>/dev/null; then s="đang chạy (pid $(cat "$LOGS/$n.pid"))"; else s="KHÔNG chạy"; fi
  echo "== $n: $s"; tail -n 4 "$LOGS/$n.log" 2>/dev/null | sed 's/^/   /'
done
echo "== tag USB:   $(ls /dev/serial/by-id/ 2>/dev/null | grep -i segger || echo 'không thấy')"
echo "== cổng FC:   $FC_PORT @ $FC_BAUD ($( [ -e "$FC_PORT" ] && echo có || echo KHÔNG THẤY))"
