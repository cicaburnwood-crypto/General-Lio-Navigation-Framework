#!/usr/bin/env bash
set -euo pipefail
deployment_dir="$(cd "$(dirname "$0")" && pwd)"
if systemctl --user is-active --quiet al-rgb-receiver.service; then
  echo 'AL RGB receiver is already running.'
  exit 0
fi
systemctl --user reset-failed al-rgb-receiver.service 2>/dev/null || true
systemd-run --user --unit=al-rgb-receiver --description='AL native RGB recording auto-download' \
  --property=Restart=on-failure --property=RestartSec=3 \
  /usr/bin/python3 -u "$deployment_dir/rgb_receiver.py" \
  --output "$deployment_dir/../recordings/AL_RGB"
