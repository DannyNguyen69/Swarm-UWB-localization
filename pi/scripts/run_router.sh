#!/bin/bash
# fc_router: owns the FC serial port, pushes telemetry to QGC over UDP and opens a local UDP port
# for uwb_bridge. Extra arguments are passed through to fc_router.
. "$(dirname "$0")/common.sh"
[ -x "$ROUTER" ] || { echo "Không thấy $ROUTER (build dự án bằng cmake trước)"; exit 1; }
exec "$ROUTER" --port "$FC_PORT" --baud "$FC_BAUD" \
     --gcs "udpout://$QGC_IP:$QGC_PORT" --local "udpin://127.0.0.1:$LOCAL_UDP" "$@"
