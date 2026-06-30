#!/usr/bin/env bash
# JetFan v2 — One-shot enable script
# Run with: bash enable.sh
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE_NAME="jetfan-v2"

echo "=== JetFan v2 Setup ==="

# 1. Python venv + dependencies
echo "[1/3] Python dependencies..."
PYTHON=$(command -v python3 || true)
if [ -z "$PYTHON" ]; then
    echo "ERROR: python3 not found. Install it first."
    exit 1
fi

if [ ! -d "$PROJECT_DIR/.venv" ]; then
    $PYTHON -m venv "$PROJECT_DIR/.venv"
fi
"$PROJECT_DIR/.venv/bin/pip" install --quiet pyserial
echo "  OK"

# 2. Create log directory
echo "[2/3] Log directory..."
mkdir -p "$PROJECT_DIR/logs"
echo "  OK"

# 3. Register systemd user service
echo "[3/3] Installing systemd user service..."

mkdir -p "$HOME/.config/systemd/user"

cat > "$HOME/.config/systemd/user/$SERVICE_NAME.service" << SERVEOF
[Unit]
Description=JetFan v2 — Temperature-based Arduino fan controller
After=graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=$PROJECT_DIR/.venv/bin/python $PROJECT_DIR/jetfan_daemon.py
WorkingDirectory=$PROJECT_DIR
Restart=on-failure
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
SERVEOF

systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"
systemctl --user start "$SERVICE_NAME"

# Enable linger so service runs before first login
loginctl enable-linger 2>/dev/null || true

echo ""
echo "=== Done! ==="
echo "  Service: $SERVICE_NAME (active, enabled at boot)"
echo "  Log: $PROJECT_DIR/logs/jetfan.log"
echo "  Status: cat $PROJECT_DIR/logs/jetfan-latest.txt"
echo "  Journal: journalctl --user -u $SERVICE_NAME -f"
