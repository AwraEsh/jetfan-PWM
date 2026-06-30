#!/usr/bin/env bash
# JetFan v2 — Disable and remove script
# Run with: bash disable.sh
set -euo pipefail

SERVICE_NAME="jetfan-v2"

echo "=== JetFan v2 Teardown ==="

# 1. Stop and disable service
echo "[1/2] Stopping and disabling service..."
systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
systemctl --user disable "$SERVICE_NAME" 2>/dev/null || true

# 2. Remove service file
echo "[2/2] Removing service file..."
rm -f "$HOME/.config/systemd/user/$SERVICE_NAME.service"
systemctl --user daemon-reload 2>/dev/null || true

echo ""
echo "=== Done! Service removed. ==="
echo "  Logs and venv are kept in place."
echo "  To fully clean: rm -rf logs/ .venv/"
