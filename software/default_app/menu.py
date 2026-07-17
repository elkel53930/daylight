#!/usr/bin/env python3
"""
menu.py — Generic cyclic menu selection.

A MenuManager knows nothing about screens or actions; it only tracks a list
of labels and the currently selected index. L moves to the next item
(cycling back to the first after the last); R is handled by the caller,
which reads `.selected`.
"""

from typing import List, Optional


class MenuManager:
    """Tracks a list of menu item labels and the current selection."""

    def __init__(self, items: Optional[List[str]] = None) -> None:
        self._items: List[str] = list(items) if items else []
        self._index: int = 0

    @property
    def items(self) -> List[str]:
        """Return the current menu item labels."""
        return list(self._items)

    @property
    def index(self) -> int:
        """Return the index of the currently selected item."""
        return self._index

    @property
    def selected(self) -> Optional[str]:
        """Return the currently selected label, or None if the menu is empty."""
        if not self._items:
            return None
        return self._items[self._index]

    def set_items(self, items: List[str]) -> None:
        """Replace the menu items and reset the selection to the first item."""
        self._items = list(items)
        self._index = 0

    def next(self) -> None:
        """Move the selection to the next item, cycling back to the first."""
        if not self._items:
            return
        self._index = (self._index + 1) % len(self._items)

    def reset(self) -> None:
        """Reset the selection to the first item."""
        self._index = 0
