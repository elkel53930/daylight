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

from ui_client import SOCKET_PATH, UIClient

NOTE_DURATION_S = 0.150  # must match ui_server's per-note duration


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <melody>", file=sys.stderr)
        sys.exit(1)

    melody = sys.argv[1]
    with UIClient() as client:
        try:
            client.connect(priority=5)
        except ConnectionError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            print(
                f"Hint: ui_server.service's socket may not be {SOCKET_PATH!r}. "
                "Check its UI_SOCKET_PATH= setting (e.g. "
                "/run/ui_server/ui_server.sock) and pass the same value "
                "here via the UI_SOCKET_PATH environment variable.",
                file=sys.stderr,
            )
            sys.exit(1)
        print(f"Playing melody: {melody!r}")
        client.play(melody)
        time.sleep(len(melody) * NOTE_DURATION_S)
        print("Done.")


if __name__ == "__main__":
    main()
