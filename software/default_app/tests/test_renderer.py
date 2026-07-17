#!/usr/bin/env python3
"""Tests for renderer.UIRenderer."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from renderer import UIRenderer, WIDTH, HEIGHT, WARN_COLOR  # noqa: E402


def _non_background_pixels(image):
    return [p for p in image.getdata() if p != (0, 0, 0)]


class TestRendererBasics(unittest.TestCase):
    def setUp(self):
        self.renderer = UIRenderer()

    def test_render_main_image_size(self):
        image = self.renderer.render_main("192.168.1.10", "BAT", "7.42V", ["Applications", "System"], 0, False)
        self.assertEqual(image.size, (WIDTH, HEIGHT))

    def test_render_main_is_rgb(self):
        image = self.renderer.render_main("192.168.1.10", "BAT", "7.42V", ["Applications", "System"], 0, False)
        self.assertEqual(image.mode, "RGB")

    def test_render_main_draws_something(self):
        image = self.renderer.render_main("192.168.1.10", "BAT", "7.42V", ["Applications", "System"], 0, False)
        self.assertTrue(_non_background_pixels(image))

    def test_render_main_menu_items_present(self):
        blank = self.renderer.render_main("N/A", "BAT", "N/A", [], 0, False)
        with_menu = self.renderer.render_main("N/A", "BAT", "N/A", ["Applications", "System"], 0, False)
        # Adding menu items should draw more non-background pixels.
        self.assertGreater(len(_non_background_pixels(with_menu)), len(_non_background_pixels(blank)))

    def test_render_main_battery_status_drawn(self):
        without = self.renderer.render_main("N/A", "BAT", "", [], 0, False)
        with_value = self.renderer.render_main("N/A", "BAT", "7.42V", [], 0, False)
        self.assertNotEqual(list(without.getdata()), list(with_value.getdata()))

    def test_render_main_low_battery_uses_warning_color(self):
        image = self.renderer.render_main("N/A", "BAT", "7.42V", [], 0, True)
        pixels = set(image.getdata())
        self.assertIn(WARN_COLOR, pixels)

    def test_render_menu_size_and_mode(self):
        image = self.renderer.render_menu("Applications", ["Maze", "Motor Test"], 0)
        self.assertEqual(image.size, (WIDTH, HEIGHT))
        self.assertEqual(image.mode, "RGB")

    def test_render_menu_empty_list(self):
        image = self.renderer.render_menu("Applications", [], 0)
        self.assertEqual(image.size, (WIDTH, HEIGHT))

    def test_render_confirm_size_and_mode(self):
        image = self.renderer.render_confirm("Reboot?")
        self.assertEqual(image.size, (WIDTH, HEIGHT))
        self.assertEqual(image.mode, "RGB")

    def test_truncate_long_label(self):
        long_name = "Very Long Application Name Indeed"
        truncated = UIRenderer._truncate(long_name)
        self.assertTrue(truncated.endswith("..."))
        self.assertLessEqual(len(truncated), 14)

    def test_truncate_short_label_unchanged(self):
        self.assertEqual(UIRenderer._truncate("Maze"), "Maze")


if __name__ == "__main__":
    unittest.main(verbosity=2)
