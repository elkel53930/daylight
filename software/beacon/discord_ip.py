#!/usr/bin/env python3
"""Discord IP notification script for Raspberry Pi startup."""

import json
import os
import platform
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

import requests

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def load_webhook_url() -> str:
    """Load Discord webhook URL from environment variable, .env file, or config.json."""
    # 1. Environment variable
    url = os.environ.get("DISCORD_WEBHOOK_URL")
    if url:
        return url

    # 2. .env file
    env_path = Path(__file__).parent / ".env"
    if env_path.exists():
        with env_path.open() as f:
            for line in f:
                line = line.strip()
                if line.startswith("DISCORD_WEBHOOK_URL="):
                    url = line.split("=", 1)[1].strip().strip('"').strip("'")
                    if url:
                        return url

    # 3. config.json
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        with config_path.open() as f:
            config = json.load(f)
        url = config.get("DISCORD_WEBHOOK_URL", "")
        if url:
            return url

    print("ERROR: DISCORD_WEBHOOK_URL is not set.", file=sys.stderr)
    print(
        "Set it via environment variable, .env file, or config.json.",
        file=sys.stderr,
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# Network helpers
# ---------------------------------------------------------------------------

INTERNET_CHECK_HOST = "8.8.8.8"
INTERNET_CHECK_PORT = 53
RETRY_INTERVAL = 5  # seconds


def is_internet_available() -> bool:
    """Return True if TCP connection to Google DNS succeeds."""
    try:
        with socket.create_connection((INTERNET_CHECK_HOST, INTERNET_CHECK_PORT), timeout=5):
            return True
    except OSError:
        return False


def wait_for_network() -> None:
    """Block until internet connectivity is confirmed."""
    print("Waiting for network...")
    while not is_internet_available():
        time.sleep(RETRY_INTERVAL)
    print("Network connected.")


# ---------------------------------------------------------------------------
# System information
# ---------------------------------------------------------------------------

def get_local_ip() -> str:
    """Return the primary local IPv4 address."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "Unknown"


def get_global_ip() -> str:
    """Return the global IPv4 address via api.ipify.org."""
    try:
        response = requests.get("https://api.ipify.org", timeout=10)
        response.raise_for_status()
        return response.text.strip()
    except requests.RequestException as e:
        print(f"WARNING: Could not retrieve global IP: {e}", file=sys.stderr)
        return "Unknown"


def get_hostname() -> str:
    return socket.gethostname()


def get_kernel_version() -> str:
    try:
        return platform.release()
    except Exception:
        return "Unknown"


def get_os_info() -> str:
    """Return Raspberry Pi OS version string from /etc/os-release."""
    try:
        os_release = Path("/etc/os-release")
        if os_release.exists():
            info = {}
            with os_release.open() as f:
                for line in f:
                    if "=" in line:
                        k, v = line.strip().split("=", 1)
                        info[k] = v.strip('"')
            pretty = info.get("PRETTY_NAME", "")
            if pretty:
                return pretty
    except Exception:
        pass
    return platform.platform()


def get_ssid() -> str:
    """Return connected Wi-Fi SSID using iwgetid."""
    try:
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        ssid = result.stdout.strip()
        return ssid if ssid else "Unknown"
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.SubprocessError):
        return "Unknown"


def get_jst_now() -> str:
    """Return current datetime formatted in JST."""
    jst = timezone(timedelta(hours=9))
    return datetime.now(jst).strftime("%Y-%m-%d %H:%M:%S JST")


# ---------------------------------------------------------------------------
# Discord notification
# ---------------------------------------------------------------------------

def build_message() -> str:
    hostname = get_hostname()
    local_ip = get_local_ip()
    global_ip = get_global_ip()
    now = get_jst_now()
    kernel = get_kernel_version()
    os_info = get_os_info()
    ssid = get_ssid()

    lines = [
        "🟢 **Raspberry Pi 起動**",
        "",
        f"**Host:** `{hostname}`",
        "",
        f"**Time:**\n{now}",
        "",
        f"**Local IP:**\n`{local_ip}`",
        "",
        f"**Global IP:**\n`{global_ip}`",
        "",
        f"**Kernel:** `{kernel}`",
        f"**OS:** {os_info}",
        f"**SSID:** `{ssid}`",
    ]
    return "\n".join(lines)


def send_notification(webhook_url: str) -> None:
    """Send a Discord webhook notification."""
    print("Sending Discord notification...")
    payload = {"content": build_message()}
    response = requests.post(webhook_url, json=payload, timeout=15)
    response.raise_for_status()
    print("Notification sent successfully.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    webhook_url = load_webhook_url()
    wait_for_network()
    try:
        send_notification(webhook_url)
    except requests.RequestException as e:
        print(f"ERROR: Failed to send Discord notification: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
