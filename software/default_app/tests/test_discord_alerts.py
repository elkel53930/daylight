#!/usr/bin/env python3
"""Tests for discord_alerts.DiscordAlertMonitor (network mocked)."""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).parent.parent))

from discord_alerts import DiscordAlertMonitor, _load_webhook_url_safe  # noqa: E402


def _make_monitor(**overrides) -> DiscordAlertMonitor:
    battery = MagicMock()
    battery.is_low = False
    battery.voltage = 7.4

    system_info = MagicMock()
    system_info.get_cpu_temp.return_value = 50.0

    defaults = dict(
        battery=battery,
        system_info=system_info,
        webhook_url="https://discord.example/webhook",
    )
    defaults.update(overrides)
    return DiscordAlertMonitor(**defaults)


class TestBatteryAlert(unittest.TestCase):
    @patch("discord_alerts.requests.post")
    def test_low_battery_sends_alert(self, mock_post):
        monitor = _make_monitor()
        monitor._battery.is_low = True
        monitor._battery.voltage = 6.1

        monitor.check_once()

        mock_post.assert_called_once()
        payload = mock_post.call_args.kwargs["json"]
        self.assertIn("6.10V", payload["content"])

    @patch("discord_alerts.requests.post")
    def test_alert_fires_only_once_while_still_low(self, mock_post):
        monitor = _make_monitor()
        monitor._battery.is_low = True

        monitor.check_once()
        monitor.check_once()

        mock_post.assert_called_once()

    @patch("discord_alerts.requests.post")
    def test_alert_fires_again_after_recovery(self, mock_post):
        monitor = _make_monitor()
        monitor._battery.is_low = True
        monitor.check_once()

        monitor._battery.is_low = False
        monitor.check_once()

        monitor._battery.is_low = True
        monitor.check_once()

        self.assertEqual(mock_post.call_count, 2)

    @patch("discord_alerts.requests.post")
    def test_normal_voltage_sends_nothing(self, mock_post):
        monitor = _make_monitor()
        monitor.check_once()
        mock_post.assert_not_called()


class TestCpuTempAlert(unittest.TestCase):
    @patch("discord_alerts.requests.post")
    def test_high_temp_sends_alert(self, mock_post):
        monitor = _make_monitor()
        monitor._system_info.get_cpu_temp.return_value = 80.0

        monitor.check_once()

        mock_post.assert_called_once()
        payload = mock_post.call_args.kwargs["json"]
        self.assertIn("80.0C", payload["content"])

    @patch("discord_alerts.requests.post")
    def test_hysteresis_stays_alerted_until_clear_threshold(self, mock_post):
        monitor = _make_monitor()

        monitor._system_info.get_cpu_temp.return_value = 80.0
        monitor.check_once()

        # 73C is below the 75C threshold but above the 72C clear
        # threshold: should stay alerted, so no second alert yet.
        monitor._system_info.get_cpu_temp.return_value = 73.0
        monitor.check_once()

        monitor._system_info.get_cpu_temp.return_value = 80.0
        monitor.check_once()
        self.assertEqual(mock_post.call_count, 1)

        # Drop to/below the clear threshold, then exceed again: new alert.
        monitor._system_info.get_cpu_temp.return_value = 72.0
        monitor.check_once()
        monitor._system_info.get_cpu_temp.return_value = 80.0
        monitor.check_once()
        self.assertEqual(mock_post.call_count, 2)

    @patch("discord_alerts.requests.post")
    def test_temp_unavailable_sends_nothing(self, mock_post):
        monitor = _make_monitor()
        monitor._system_info.get_cpu_temp.return_value = None
        monitor.check_once()
        mock_post.assert_not_called()


class TestWebhookMissing(unittest.TestCase):
    @patch("discord_alerts._load_webhook_url_safe", return_value=None)
    @patch("discord_alerts.requests.post")
    def test_no_webhook_url_skips_send_without_raising(self, mock_post, _mock_load):
        # Patch the module-level lookup (rather than relying on the host
        # having no DISCORD_WEBHOOK_URL configured) so this is
        # deterministic regardless of the machine running the tests.
        monitor = _make_monitor(webhook_url=None)
        monitor._battery.is_low = True

        monitor.check_once()

        mock_post.assert_not_called()

    @patch("discord_alerts.load_webhook_url", side_effect=SystemExit(1))
    def test_load_webhook_url_safe_swallows_sys_exit(self, _mock_load):
        # load_webhook_url() calls sys.exit() when unconfigured; callers
        # must not have that propagate as a process-ending crash.
        self.assertIsNone(_load_webhook_url_safe())


if __name__ == "__main__":
    unittest.main()
