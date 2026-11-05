#!/bin/bash
# uwb_bridge: UWB tag -> position -> VISION_POSITION_ESTIMATE to the FC through fc_router.
# Uses the C++ binary (cpp/build/uwb_bridge) when it exists, otherwise the Python one; USE_PY=1 forces Python.
# Extra arguments are passed through.
. "$(dirname "$0")/common.sh"
[ -n "$HEADING" ] || { echo "Chưa đặt HEADING trong $ROOT/config/uwb.env."; echo "Đo bằng scripts/measure_heading.sh (drone nằm yên, mũi hướng từ A1 sang A2)."; exit 1; }
BIN="$ROOT/cpp/build/uwb_bridge"
# Anchors are re-hung every session, so by default only this session's survey is used (REQUIRE_SURVEY=0: anchors.toml).
EXTRA=()
[ "${REQUIRE_SURVEY:-1}" = 1 ] && EXTRA+=(--require-survey)
if [ -x "$BIN" ] && [ -z "$USE_PY" ]; then
    exec "$BIN" --fc-url "udpout://127.0.0.1:$LOCAL_UDP" --heading "$HEADING" --origin "$ORIGIN" \
         --tag-z "$TAG_Z" --rate "$RATE" --anchors "$ROOT/config/anchors.toml" --raw-dir "$LOGS/uwb_raw" "${EXTRA[@]}" "$@"
fi
exec "$PY" -u "$ROOT/uwb/uwb_bridge.py" --fw ardupilot --fc "udpout:127.0.0.1:$LOCAL_UDP" \
     --heading "$HEADING" --origin "$ORIGIN" --tag-z "$TAG_Z" --rate "$RATE" \
     --anchors "$ROOT/config/anchors.toml" "$@"
