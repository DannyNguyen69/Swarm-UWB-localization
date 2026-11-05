#!/bin/bash
# NUC -> Pi. Copies this folder into ~/dark-landing on the Pi, plus the shared modules from host/.
#   pi/sync.sh                  # to pi4@pi4.local
#   pi/sync.sh pi4@192.168.0.21
# With a password login, export SSHPASS first (needs sshpass); with an SSH key nothing is needed.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"
TARGET="${1:-pi4@pi4.local}"
DEST="dark-landing"
SSH_PASS=""; [ -n "$SSHPASS" ] && SSH_PASS="sshpass -e"
SSH="$SSH_PASS ssh -o StrictHostKeyChecking=accept-new"

# 1. this folder (scripts, uwb, systemd, docs, config); the Pi's own uwb.env, logs and venv stay
rsync -av -e "$SSH" --exclude '__pycache__' --exclude '.venv' --exclude 'logs' --exclude 'sync.sh' \
      --exclude 'config/uwb.env' "$HERE"/ "$TARGET:$DEST/"
# 2. uwb.env only the first time, so edits made on the Pi (HEADING, ORIGIN) are never overwritten
rsync -av -e "$SSH" --ignore-existing "$HERE/config/uwb.env" "$TARGET:$DEST/config/uwb.env"
# 3. shared modules and the anchor file from host/ (single source of truth on the NUC)
$SSH "$TARGET" "mkdir -p $DEST/uwb/lib"
rsync -av -e "$SSH" "$REPO/host/uwbproto.py" "$REPO/host/survey_solver.py" "$REPO/host/uwb_viewer.py" \
      "$TARGET:$DEST/uwb/lib/"
rsync -av -e "$SSH" "$REPO/host/anchors.toml" "$TARGET:$DEST/config/anchors.toml"
echo "xong. Trên Pi: cd ~/$DEST && scripts/setup_pi.sh   (lần đầu)"
