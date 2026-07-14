#!/usr/bin/env python3
"""
melody_test.py — Play a sample melody via ui_server.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from ui_client import UIClient

SAMPLE_MELODY = "g_gC__D_EC__a_gC__C__C"


def main() -> None:
    with UIClient() as client:
        client.connect(priority=5)
        print(f"Playing melody: {SAMPLE_MELODY!r}")
        client.play(SAMPLE_MELODY)
        print("Done.")


if __name__ == "__main__":
    main()
