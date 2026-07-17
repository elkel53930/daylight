#!/usr/bin/env python3
"""
renderer.py — Builds 96x64 RGB PIL images for each Default UI screen.

This module has no knowledge of ui_server, sockets, or application state; it
only turns plain data into images that can be handed to ui_client.display().
"""

import logging
from typing import List, Tuple

from PIL import Image, ImageDraw, ImageFont

logger = logging.getLogger("default_ui.renderer")

WIDTH = 96
HEIGHT = 64

DEFAULT_FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
DEFAULT_FONT_SIZE = 10
LINE_HEIGHT = 12

BG_COLOR = (0, 0, 0)
FG_COLOR = (255, 255, 255)
WARN_COLOR = (255, 40, 40)

MAX_LABEL_CHARS = 14


class UIRenderer:
    """Renders Default UI screens as PIL.Image (RGB, 96x64)."""

    def __init__(self, font_path: str = DEFAULT_FONT_PATH, font_size: int = DEFAULT_FONT_SIZE) -> None:
        try:
            self._font = ImageFont.truetype(font_path, font_size)
        except (OSError, IOError) as exc:
            logger.warning("Failed to load font %s (%s); using default font", font_path, exc)
            self._font = ImageFont.load_default()

    def _blank(self) -> Tuple[Image.Image, ImageDraw.ImageDraw]:
        image = Image.new("RGB", (WIDTH, HEIGHT), BG_COLOR)
        return image, ImageDraw.Draw(image)

    def render_main(
        self,
        ip_text: str,
        status_label: str,
        status_value: str,
        menu_items: List[str],
        selected_index: int,
        low_battery: bool = False,
    ) -> Image.Image:
        """Render the home screen: IP, rotating status line, and the main menu."""
        image, draw = self._blank()
        draw.text((0, 0), f"IP: {ip_text}", fill=FG_COLOR, font=self._font)

        if low_battery:
            draw.text((0, LINE_HEIGHT), "BATTERY LOW", fill=WARN_COLOR, font=self._font)
        else:
            draw.text((0, LINE_HEIGHT), f"{status_label}: {status_value}", fill=FG_COLOR, font=self._font)

        y = LINE_HEIGHT * 3
        for i, item in enumerate(menu_items):
            prefix = "> " if i == selected_index else "  "
            draw.text((0, y), prefix + self._truncate(item), fill=FG_COLOR, font=self._font)
            y += LINE_HEIGHT
            if y >= HEIGHT:
                break
        return image

    def render_menu(self, title: str, items: List[str], selected_index: int) -> Image.Image:
        """Render a submenu screen (e.g. Applications, System) with a highlighted selection."""
        image, draw = self._blank()
        draw.text((0, 0), title, fill=FG_COLOR, font=self._font)

        y = LINE_HEIGHT * 2
        if not items:
            draw.text((0, y), "(empty)", fill=FG_COLOR, font=self._font)
            return image

        for i, item in enumerate(items):
            label = self._truncate(item)
            if i == selected_index:
                draw.rectangle([(0, y), (WIDTH, y + LINE_HEIGHT - 1)], fill=FG_COLOR)
                draw.text((2, y), label, fill=BG_COLOR, font=self._font)
            else:
                draw.text((2, y), label, fill=FG_COLOR, font=self._font)
            y += LINE_HEIGHT
            if y >= HEIGHT:
                break
        return image

    def render_confirm(self, question: str) -> Image.Image:
        """Render a Yes/No confirmation screen (L: No, R: Yes)."""
        image, draw = self._blank()
        draw.text((0, 0), question, fill=FG_COLOR, font=self._font)
        draw.text((0, LINE_HEIGHT * 3), "L: No", fill=FG_COLOR, font=self._font)
        draw.text((0, LINE_HEIGHT * 4), "R: Yes", fill=FG_COLOR, font=self._font)
        return image

    @staticmethod
    def _truncate(text: str, max_chars: int = MAX_LABEL_CHARS) -> str:
        if len(text) <= max_chars:
            return text
        return text[: max_chars - 3] + "..."
