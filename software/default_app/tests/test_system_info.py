#!/usr/bin/env python3
"""Tests for system_info.SystemInfo."""

import socket
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock

sys.path.insert(0, str(Path(__file__).parent.parent))

from system_info import SystemInfo  # noqa: E402


class TestCpuTemp(unittest.TestCase):
    def test_parses_normal_output(self):
        info = SystemInfo()
        completed = MagicMock(stdout="temp=48.7'C\n")
        with patch("system_info.subprocess.run", return_value=completed):
            self.assertEqual(info.get_cpu_temp(), 48.7)

    def test_vcgencmd_failure_returns_none(self):
        info = SystemInfo()
        with patch(
            "system_info.subprocess.run",
            side_effect=subprocess.SubprocessError("boom"),
        ):
            self.assertIsNone(info.get_cpu_temp())

    def test_vcgencmd_missing_returns_none(self):
        info = SystemInfo()
        with patch("system_info.subprocess.run", side_effect=FileNotFoundError()):
            self.assertIsNone(info.get_cpu_temp())

    def test_unparseable_output_returns_none(self):
        info = SystemInfo()
        completed = MagicMock(stdout="garbage output")
        with patch("system_info.subprocess.run", return_value=completed):
            self.assertIsNone(info.get_cpu_temp())


class TestCpuFreq(unittest.TestCase):
    def test_converts_khz_to_mhz(self):
        info = SystemInfo()
        completed = MagicMock(stdout="1500000\n")
        with patch("system_info.subprocess.run", return_value=completed):
            self.assertEqual(info.get_cpu_freq_mhz(), 1500)

    def test_sudo_cat_failure_returns_none(self):
        info = SystemInfo()
        with patch(
            "system_info.subprocess.run",
            side_effect=subprocess.SubprocessError("boom"),
        ):
            self.assertIsNone(info.get_cpu_freq_mhz())

    def test_sudo_missing_returns_none(self):
        info = SystemInfo()
        with patch("system_info.subprocess.run", side_effect=FileNotFoundError()):
            self.assertIsNone(info.get_cpu_freq_mhz())

    def test_unparseable_content_returns_none(self):
        info = SystemInfo()
        completed = MagicMock(stdout="not-a-number")
        with patch("system_info.subprocess.run", return_value=completed):
            self.assertIsNone(info.get_cpu_freq_mhz())


class TestWifiIp(unittest.TestCase):
    def test_returns_ip_address(self):
        info = SystemInfo(interface="wlan0")
        packed = socket.inet_aton("192.168.1.10")
        fake_response = b"\x00" * 20 + packed
        with patch("system_info.fcntl.ioctl", return_value=fake_response):
            self.assertEqual(info.get_ip_address(), "192.168.1.10")

    def test_missing_interface_returns_none(self):
        info = SystemInfo(interface="wlan0")
        with patch("system_info.fcntl.ioctl", side_effect=OSError("no such device")):
            self.assertIsNone(info.get_ip_address())


if __name__ == "__main__":
    unittest.main(verbosity=2)
