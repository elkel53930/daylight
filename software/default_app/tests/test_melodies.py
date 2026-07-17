#!/usr/bin/env python3
"""
Tests for melodies.py.

ui_server treats any character outside VALID_NOTES as a silent rest, so a
typo here wouldn't raise an error — it would just go quiet. These tests
catch that class of mistake.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import melodies  # noqa: E402

VALID_NOTES = set("cdefgabCDEFGAB")


class TestMelodiesAreValid(unittest.TestCase):
    def test_button_click_melodies_use_valid_notes(self):
        for name, melody in melodies.BUTTON_CLICK_MELODY.items():
            self.assertTrue(melody, f"{name} click melody is empty")
            self.assertTrue(
                set(melody) <= VALID_NOTES,
                f"{name} click melody {melody!r} contains a non-note character",
            )

    def test_app_launch_melody_uses_valid_notes(self):
        self.assertTrue(melodies.APP_LAUNCH_MELODY)
        self.assertTrue(set(melodies.APP_LAUNCH_MELODY) <= VALID_NOTES)

    def test_low_battery_melody_uses_valid_notes(self):
        self.assertTrue(melodies.LOW_BATTERY_MELODY)
        self.assertTrue(set(melodies.LOW_BATTERY_MELODY) <= VALID_NOTES)

    def test_button_click_melody_has_both_buttons(self):
        self.assertEqual(set(melodies.BUTTON_CLICK_MELODY.keys()), {"left", "right"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
