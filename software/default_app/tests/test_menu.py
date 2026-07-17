#!/usr/bin/env python3
"""Tests for menu.MenuManager."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from menu import MenuManager  # noqa: E402


class TestMenuManager(unittest.TestCase):
    def test_initial_selection_is_first_item(self):
        menu = MenuManager(["A", "B", "C"])
        self.assertEqual(menu.selected, "A")
        self.assertEqual(menu.index, 0)

    def test_next_moves_forward(self):
        menu = MenuManager(["A", "B", "C"])
        menu.next()
        self.assertEqual(menu.selected, "B")
        menu.next()
        self.assertEqual(menu.selected, "C")

    def test_next_cycles_back_to_first(self):
        menu = MenuManager(["A", "B", "C"])
        menu.next()
        menu.next()
        menu.next()
        self.assertEqual(menu.selected, "A")
        self.assertEqual(menu.index, 0)

    def test_empty_menu_selected_is_none(self):
        menu = MenuManager([])
        self.assertIsNone(menu.selected)

    def test_empty_menu_next_is_noop(self):
        menu = MenuManager([])
        menu.next()
        self.assertIsNone(menu.selected)
        self.assertEqual(menu.index, 0)

    def test_single_item_menu_stays_on_same_item(self):
        menu = MenuManager(["Only"])
        menu.next()
        self.assertEqual(menu.selected, "Only")
        self.assertEqual(menu.index, 0)

    def test_set_items_resets_selection(self):
        menu = MenuManager(["A", "B"])
        menu.next()
        self.assertEqual(menu.index, 1)
        menu.set_items(["X", "Y", "Z"])
        self.assertEqual(menu.index, 0)
        self.assertEqual(menu.selected, "X")

    def test_reset(self):
        menu = MenuManager(["A", "B"])
        menu.next()
        menu.reset()
        self.assertEqual(menu.index, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
