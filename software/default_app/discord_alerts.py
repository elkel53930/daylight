#!/usr/bin/env python3
"""
discord_alerts.py — Watches battery voltage and CPU temperature in its own
background thread and posts a Discord webhook notification when either
crosses into a warning state (with hysteresis, so it doesn't spam once per
poll while hovering near the threshold).

Runs on a thread independent of the ui_server connection and the main UI
loop, so alerts keep firing even while DefaultUI is disconnected — e.g.
while blocked waiting for a launched application to exit (see
DefaultUI._launch_app).

Webhook URL loading follows the same convention as camera_discord.py /
beacon/discord_ip.py (env var, beacon/.env, or beacon/config.json).
"""

import logging
import sys
from pathlib import Path
from typing import Optional

import requests

from battery import BatteryMonitor
from hysteresis import Hysteresis
from polling_thread import PollingThread
from system_info import SystemInfo

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "beacon"))
from discord_ip import load_webhook_url  # noqa: E402

logger = logging.getLogger("default_ui.discord_alerts")

DEFAULT_POLL_INTERVAL_S = 10.0
DEFAULT_CPU_HIGH_THRESHOLD = 75.0
DEFAULT_CPU_HIGH_CLEAR_THRESHOLD = 72.0


def _load_webhook_url_safe() -> Optional[str]:
    """Like discord_ip.load_webhook_url(), but returns None instead of
    calling sys.exit() when no webhook URL is configured."""
    try:
        return load_webhook_url()
    except SystemExit:
        return None


class DiscordAlertMonitor(PollingThread):
    """
    Periodically checks battery low-voltage state and CPU temperature and
    posts a Discord notification on each transition into a warning state.

    Battery: reuses BatteryMonitor.is_low, so the 6.5V threshold (with
    6.7V hysteresis) matches the value shown on the main screen.
    CPU temperature: own threshold/clear pair below, since SystemInfo has
    no built-in hysteresis for it.
    """

    def __init__(
        self,
        battery: BatteryMonitor,
        system_info: SystemInfo,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
        cpu_high_threshold: float = DEFAULT_CPU_HIGH_THRESHOLD,
        cpu_high_clear_threshold: float = DEFAULT_CPU_HIGH_CLEAR_THRESHOLD,
        webhook_url: Optional[str] = None,
    ) -> None:
        super().__init__(poll_interval_s, thread_name="discord-alert-monitor")
        self._battery = battery
        self._system_info = system_info
        self._cpu_high_state = Hysteresis(cpu_high_threshold, cpu_high_clear_threshold, "rises_above")
        self._webhook_url = webhook_url if webhook_url is not None else _load_webhook_url_safe()

        if self._webhook_url is None:
            logger.warning("Discord webhook URL is not configured; alerts are disabled")

        self._battery_alerted = False

    def _tick(self) -> None:
        self.check_once()

    def check_once(self) -> None:
        """Run one battery + CPU temperature check. Exposed for testing."""
        self._check_battery()
        self._check_cpu_temp()

    def _check_battery(self) -> None:
        is_low = self._battery.is_low
        if is_low and not self._battery_alerted:
            self._battery_alerted = True
            voltage = self._battery.voltage
            value_text = f"{voltage:.2f}V" if voltage is not None else "unknown"
            self._send(f"\U0001F50B **バッテリー電圧低下**\n電圧: `{value_text}`")
        elif not is_low and self._battery_alerted:
            self._battery_alerted = False

    def _check_cpu_temp(self) -> None:
        temp = self._system_info.get_cpu_temp()
        if temp is None:
            return
        was_high = self._cpu_high_state.active
        is_high = self._cpu_high_state.update(temp)
        if is_high and not was_high:
            self._send(f"\U0001F321️ **CPU温度上昇**\n温度: `{temp:.1f}C`")

    def _send(self, message: str) -> None:
        if self._webhook_url is None:
            logger.warning("Skipping Discord alert (no webhook URL configured): %s", message)
            return
        try:
            response = requests.post(self._webhook_url, json={"content": message}, timeout=15)
            response.raise_for_status()
            logger.info("Sent Discord alert: %s", message.splitlines()[0])
        except requests.RequestException as exc:
            logger.warning("Failed to send Discord alert: %s", exc)
