#!/usr/bin/env python3
"""
display_image.py — Load a PNG and show it on the OLED via ui_server.

Usage:
    python3 display_image.py <image.png>
"""

import sys
from pathlib import Path

from PIL import Image

# Allow running from the examples/ directory
sys.path.insert(0, str(Path(__file__).parent.parent))
from ui_client import UIClient, DISPLAY_WIDTH, DISPLAY_HEIGHT


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <image.png>")
        sys.exit(1)

    image_path = Path(sys.argv[1])
    if not image_path.exists():
        print(f"File not found: {image_path}")
        sys.exit(1)

    img = Image.open(str(image_path)).convert("RGB").resize((DISPLAY_WIDTH, DISPLAY_HEIGHT))

    with UIClient() as client:
        client.connect(priority=5)
        client.display(img)
        print(f"Displayed {image_path.name} on OLED")


if __name__ == "__main__":
    main()
