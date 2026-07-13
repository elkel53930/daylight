#!/usr/bin/env bash
# setup.sh — Install and enable the Discord startup notification service.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="discord-startup-notify"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
CURRENT_USER="${SUDO_USER:-$(whoami)}"

# ---------------------------------------------------------------------------
# 1. venv
# ---------------------------------------------------------------------------
echo "[1/5] Checking Python venv..."
if [[ ! -d "${PROJECT_ROOT}/venv" ]]; then
    echo "  Creating venv..."
    python3 -m venv "${PROJECT_ROOT}/venv"
fi

# ---------------------------------------------------------------------------
# 2. Install dependencies
# ---------------------------------------------------------------------------
echo "[2/5] Installing Python dependencies..."
"${PROJECT_ROOT}/venv/bin/pip" install --upgrade pip -q
"${PROJECT_ROOT}/venv/bin/pip" install -r "${PROJECT_ROOT}/requirements.txt" -q
echo "  Done."

# ---------------------------------------------------------------------------
# 3. Webhook URL check
# ---------------------------------------------------------------------------
echo "[3/5] Checking webhook URL configuration..."
if [[ -z "${DISCORD_WEBHOOK_URL:-}" ]]; then
    if [[ ! -f "${PROJECT_ROOT}/.env" ]] && [[ ! -f "${PROJECT_ROOT}/config.json" ]]; then
        echo ""
        echo "  WARNING: DISCORD_WEBHOOK_URL is not configured."
        echo "  Set it using one of the following methods before starting the service:"
        echo ""
        echo "    Option A — .env file:"
        echo "      echo 'DISCORD_WEBHOOK_URL=https://discord.com/api/webhooks/...' > ${PROJECT_ROOT}/.env"
        echo ""
        echo "    Option B — config.json:"
        echo "      echo '{\"DISCORD_WEBHOOK_URL\": \"https://discord.com/api/webhooks/...\"}' > ${PROJECT_ROOT}/config.json"
        echo ""
        echo "    Option C — environment variable in the systemd service (EnvironmentFile or Environment=)."
        echo ""
    fi
fi

# ---------------------------------------------------------------------------
# 4. Generate and install systemd service
# ---------------------------------------------------------------------------
echo "[4/5] Installing systemd service to ${SERVICE_FILE}..."

if [[ "${EUID}" -ne 0 ]]; then
    echo "  ERROR: This step requires root. Please re-run with sudo:" >&2
    echo "    sudo bash ${BASH_SOURCE[0]}" >&2
    exit 1
fi

cat > "${SERVICE_FILE}" <<EOF
[Unit]
Description=Discord Startup Notification
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=${CURRENT_USER}
WorkingDirectory=${PROJECT_ROOT}
EnvironmentFile=-${PROJECT_ROOT}/.env
ExecStart=${PROJECT_ROOT}/venv/bin/python ${PROJECT_ROOT}/discord_ip.py
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "${SERVICE_FILE}"
echo "  Service file written."

# ---------------------------------------------------------------------------
# 5. Enable service
# ---------------------------------------------------------------------------
echo "[5/5] Enabling service..."
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
echo "  Service enabled."

# ---------------------------------------------------------------------------
# Usage hints
# ---------------------------------------------------------------------------
echo ""
echo "======================================================================"
echo " Setup complete!"
echo "======================================================================"
echo ""
echo " Service : ${SERVICE_NAME}"
echo " Script  : ${PROJECT_ROOT}/discord_ip.py"
echo ""
echo " Useful commands:"
echo "   Start now (test run):"
echo "     sudo systemctl start ${SERVICE_NAME}.service"
echo ""
echo "   Check logs:"
echo "     journalctl -u ${SERVICE_NAME}.service -e"
echo ""
echo "   Disable auto-start:"
echo "     sudo systemctl disable ${SERVICE_NAME}.service"
echo ""
