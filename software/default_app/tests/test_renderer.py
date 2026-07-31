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

    def _has_full_width_highlight_row(self, image):
        pixels = image.load()
        for y in range(image.height):
            if all(pixels[x, y] == (255, 255, 255) for x in range(image.width)):
                return True
        return False

    def test_render_menu_highlight_visible_for_low_selection(self):
        items = [f"Item{i}" for i in range(8)]
        image = self.renderer.render_menu("Applications", items, 0)
        self.assertTrue(self._has_full_width_highlight_row(image))

    def test_render_menu_highlight_visible_for_high_selection(self):
        # 修正前は選択が画面に収まる項目数を超えると、選択行が一切
        # 描画されなくなっていた(常に先頭から描画していたため)。
        items = [f"Item{i}" for i in range(8)]
        image = self.renderer.render_menu("Applications", items, 7)
        self.assertTrue(self._has_full_width_highlight_row(image))

    def test_render_menu_scrolls_as_selection_moves(self):
        items = [f"Item{i}" for i in range(8)]
        low = self.renderer.render_menu("Applications", items, 0)
        high = self.renderer.render_menu("Applications", items, 7)
        self.assertNotEqual(list(low.getdata()), list(high.getdata()))

    def test_render_menu_few_items_no_crash(self):
        # 項目数が可視数以下ならスクロール計算が単純化されるだけで動作は変わらない
        image = self.renderer.render_menu("Applications", ["Maze"], 0)
        self.assertEqual(image.size, (WIDTH, HEIGHT))


class TestScrollStart(unittest.TestCase):
    def test_all_items_fit_no_scroll(self):
        self.assertEqual(UIRenderer._scroll_start(0, 3, 3), 0)
        self.assertEqual(UIRenderer._scroll_start(2, 3, 3), 0)
        self.assertEqual(UIRenderer._scroll_start(0, 1, 3), 0)

    def test_low_selection_clamped_to_start(self):
        self.assertEqual(UIRenderer._scroll_start(0, 8, 3), 0)
        self.assertEqual(UIRenderer._scroll_start(1, 8, 3), 0)

    def test_high_selection_clamped_to_end(self):
        self.assertEqual(UIRenderer._scroll_start(6, 8, 3), 5)
        self.assertEqual(UIRenderer._scroll_start(7, 8, 3), 5)

    def test_middle_selection_centers_window(self):
        self.assertEqual(UIRenderer._scroll_start(4, 8, 3), 3)


class TestRendererMisc(unittest.TestCase):
    def setUp(self):
        self.renderer = UIRenderer()

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
