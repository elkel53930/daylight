#!/usr/bin/env python3
"""
button_test.py — Poll and print button states every 100 ms.

Press Ctrl+C to stop.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from ui_client import UIClient


def main() -> None:
    with UIClient() as client:
        client.connect(priority=5)
        print("Monitoring buttons (Ctrl+C to stop) ...")
        try:
            while True:
                buttons = client.get_buttons()
                left = buttons.get("left", "?")
                right = buttons.get("right", "?")
                print(f"\rLEFT: {left:<12}  RIGHT: {right:<12}", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
