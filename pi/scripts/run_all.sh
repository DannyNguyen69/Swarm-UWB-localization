#!/bin/bash
# Starts fc_router and uwb_bridge in the background; logs in logs/, PIDs in logs/*.pid.
#   scripts/run_all.sh          start both
#   scripts/stop_all.sh         stop both
#   scripts/status.sh           state and last log lines
. "$(dirname "$0")/common.sh"
for n in router uwb; do
  if [ -f "$LOGS/$n.pid" ] && kill -0 "$(cat "$LOGS/$n.pid")" 2>/dev/null; then
    echo "$n đang chạy (pid $(cat "$LOGS/$n.pid")). Chạy scripts/stop_all.sh trước."; exit 1
  fi
done
[ -n "$HEADING" ] || { echo "Chưa đặt HEADING trong $ROOT/config/uwb.env (xem scripts/measure_heading.sh)."; exit 1; }

nohup "$ROOT/scripts/run_router.sh" > "$LOGS/router.log" 2>&1 &
echo $! > "$LOGS/router.pid"
for i in $(seq 1 20); do grep -q "FC sysid" "$LOGS/router.log" 2>/dev/null && break; sleep 0.5; done
grep -q "FC sysid" "$LOGS/router.log" || echo "Cảnh báo: router chưa thấy FC (FC đã cấp nguồn chưa?). Vẫn chạy tiếp, router sẽ chờ."

nohup "$ROOT/scripts/run_uwb.sh" > "$LOGS/uwb.log" 2>&1 &
echo $! > "$LOGS/uwb.pid"
echo "đã chạy: router pid $(cat "$LOGS/router.pid"), uwb pid $(cat "$LOGS/uwb.pid")"
echo "xem log:  tail -f $LOGS/uwb.log     trạng thái: scripts/status.sh"
