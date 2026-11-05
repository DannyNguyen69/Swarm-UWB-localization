#!/bin/bash
# Installs and enables dl-router + dl-uwb so the Pi starts the UDP link and the UWB feed at power-on.
# Run once on the Pi after building (scripts/build_uwb.sh) and setting HEADING in config/uwb.env.
. "$(dirname "$0")/common.sh"
[ -n "$HEADING" ] || { echo "Đặt HEADING trong config/uwb.env trước đã."; exit 1; }
[ -x "$ROOT/cpp/build/uwb_bridge" ] || "$ROOT/scripts/build_uwb.sh"
sudo cp "$ROOT"/systemd/dl-*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dl-router dl-uwb
echo "Đã bật. Xem log: journalctl -u dl-router -u dl-uwb -f"
echo "Tắt tự chạy: sudo systemctl disable --now dl-uwb dl-router"
