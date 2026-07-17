#!/usr/bin/env python3
"""Tests for hysteresis.Hysteresis."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from hysteresis import Hysteresis  # noqa: E402


class TestFallsBelow(unittest.TestCase):
    """direction="falls_below": mirrors battery low-voltage semantics."""

    def test_starts_inactive(self):
        h = Hysteresis(6.5, 6.7, "falls_below")
        self.assertFalse(h.active)

    def test_value_above_threshold_stays_inactive(self):
        h = Hysteresis(6.5, 6.7, "falls_below")
        self.assertFalse(h.update(7.4))

    def test_value_below_threshold_activates(self):
        h = Hysteresis(6.5, 6.7, "falls_below")
        self.assertTrue(h.update(6.0))

    def test_stays_active_until_clear_threshold(self):
        h = Hysteresis(6.5, 6.7, "falls_below")
        h.update(6.0)
        self.assertTrue(h.active)

        # 6.6V is above the 6.5V threshold but below the 6.7V clear
        # threshold: must remain active (hysteresis).
        self.assertTrue(h.update(6.6))

        self.assertFalse(h.update(6.8))

    def test_reactivates_after_clearing(self):
        h = Hysteresis(6.5, 6.7, "falls_below")
        h.update(6.0)
        h.update(6.8)
        self.assertFalse(h.active)
        self.assertTrue(h.update(6.0))


class TestRisesAbove(unittest.TestCase):
    """direction="rises_above": mirrors CPU high-temperature semantics."""

    def test_starts_inactive(self):
        h = Hysteresis(75.0, 72.0, "rises_above")
        self.assertFalse(h.active)

    def test_value_below_threshold_stays_inactive(self):
        h = Hysteresis(75.0, 72.0, "rises_above")
        self.assertFalse(h.update(50.0))

    def test_value_above_threshold_activates(self):
        h = Hysteresis(75.0, 72.0, "rises_above")
        self.assertTrue(h.update(80.0))

    def test_stays_active_until_clear_threshold(self):
        h = Hysteresis(75.0, 72.0, "rises_above")
        h.update(80.0)
        self.assertTrue(h.active)

        # 73C is below the 75C threshold but above the 72C clear
        # threshold: must remain active (hysteresis).
        self.assertTrue(h.update(73.0))

        self.assertFalse(h.update(72.0))

    def test_reactivates_after_clearing(self):
        h = Hysteresis(75.0, 72.0, "rises_above")
        h.update(80.0)
        h.update(72.0)
        self.assertFalse(h.active)
        self.assertTrue(h.update(80.0))


class TestInvalidDirection(unittest.TestCase):
    def test_rejects_unknown_direction(self):
        with self.assertRaises(ValueError):
            Hysteresis(1.0, 2.0, "sideways")


if __name__ == "__main__":
    unittest.main()
