#!/usr/bin/env python3
"""Tests for battery.BatteryMonitor and MCP3221Reader (hardware mocked)."""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).parent.parent))

from battery import BatteryMonitor  # noqa: E402


class TestRawToVoltage(unittest.TestCase):
    def test_zero_raw_value(self):
        self.assertAlmostEqual(BatteryMonitor.raw_to_voltage(0), 0.0)

    def test_max_raw_value(self):
        # 4095 / 4095 * 3.3 * 11 = 36.3
        self.assertAlmostEqual(BatteryMonitor.raw_to_voltage(4095), 36.3, places=3)

    def test_typical_voltage(self):
        # Raw value corresponding to roughly 7.4V
        raw = round(7.4 / (3.3 * 11) * 4095)
        voltage = BatteryMonitor.raw_to_voltage(raw)
        self.assertAlmostEqual(voltage, 7.4, delta=0.01)


class TestBatteryMonitorPolling(unittest.TestCase):
    def _make_monitor(self, raw_value: int) -> BatteryMonitor:
        reader = MagicMock()
        reader.read_raw.return_value = raw_value
        return BatteryMonitor(reader=reader)

    def test_poll_once_updates_voltage(self):
        monitor = self._make_monitor(raw_value=2048)
        voltage = monitor.poll_once()
        self.assertIsNotNone(voltage)
        self.assertEqual(monitor.voltage, voltage)

    def test_normal_voltage_is_not_low(self):
        # ~7.4V raw value
        raw = round(7.4 / (3.3 * 11) * 4095)
        monitor = self._make_monitor(raw_value=raw)
        monitor.poll_once()
        self.assertFalse(monitor.is_low)

    def test_voltage_below_threshold_is_low(self):
        # ~6.0V raw value (below 6.5V threshold)
        raw = round(6.0 / (3.3 * 11) * 4095)
        monitor = self._make_monitor(raw_value=raw)
        monitor.poll_once()
        self.assertTrue(monitor.is_low)

    def test_hysteresis_stays_low_until_clear_threshold(self):
        monitor = self._make_monitor(raw_value=0)

        low_raw = round(6.0 / (3.3 * 11) * 4095)
        monitor._reader.read_raw.return_value = low_raw
        monitor.poll_once()
        self.assertTrue(monitor.is_low)

        # 6.6V is above the 6.5V low threshold but below the 6.7V clear
        # threshold: should remain in the low state (hysteresis).
        mid_raw = round(6.6 / (3.3 * 11) * 4095)
        monitor._reader.read_raw.return_value = mid_raw
        monitor.poll_once()
        self.assertTrue(monitor.is_low)

        # 6.8V clears the low state.
        clear_raw = round(6.8 / (3.3 * 11) * 4095)
        monitor._reader.read_raw.return_value = clear_raw
        monitor.poll_once()
        self.assertFalse(monitor.is_low)

    def test_i2c_failure_returns_none_and_keeps_previous_state(self):
        monitor = self._make_monitor(raw_value=2048)
        monitor.poll_once()
        previous_voltage = monitor.voltage

        monitor._reader.read_raw.side_effect = OSError("i2c error")
        result = monitor.poll_once()

        self.assertIsNone(result)
        self.assertEqual(monitor.voltage, previous_voltage)


if __name__ == "__main__":
    unittest.main(verbosity=2)
