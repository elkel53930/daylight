#!/usr/bin/env python3
"""
try_melody.py — Play a melody given as a command-line argument.

Usage:
    python3 try_melody.py <melody>

Example:
    python3 try_melody.py ccddeeff
"""

import sys
import time

from ui_client import UIClient

NOTE_DURATION_S = 0.150  # must match ui_server's per-note duration


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <melody>", file=sys.stderr)
        sys.exit(1)

    melody = sys.argv[1]
    with UIClient() as client:
        client.connect(priority=5)
        print(f"Playing melody: {melody!r}")
        client.play(melody)
        time.sleep(len(melody) * NOTE_DURATION_S)
        print("Done.")


if __name__ == "__main__":
    main()
