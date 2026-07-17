#!/usr/bin/env python3
"""Tests for polling_thread.PollingThread."""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from polling_thread import PollingThread  # noqa: E402

POLL_INTERVAL_S = 0.02


class _CountingThread(PollingThread):
    def __init__(self, poll_interval_s: float = POLL_INTERVAL_S) -> None:
        super().__init__(poll_interval_s, thread_name="test-polling-thread")
        self.tick_count = 0

    def _tick(self) -> None:
        self.tick_count += 1


class TestPollingThread(unittest.TestCase):
    def test_start_calls_tick_repeatedly(self):
        thread = _CountingThread()
        thread.start()
        try:
            time.sleep(POLL_INTERVAL_S * 10)
            self.assertGreaterEqual(thread.tick_count, 3)
        finally:
            thread.stop()

    def test_stop_halts_ticking(self):
        thread = _CountingThread()
        thread.start()
        time.sleep(POLL_INTERVAL_S * 5)
        thread.stop()
        count_after_stop = thread.tick_count

        time.sleep(POLL_INTERVAL_S * 5)
        self.assertEqual(thread.tick_count, count_after_stop)

    def test_start_is_idempotent(self):
        thread = _CountingThread()
        thread.start()
        first_thread = thread._thread
        thread.start()
        try:
            self.assertIs(thread._thread, first_thread)
        finally:
            thread.stop()

    def test_stop_before_start_is_a_noop(self):
        thread = _CountingThread()
        thread.stop()  # must not raise
        self.assertEqual(thread.tick_count, 0)

    def test_unimplemented_tick_raises(self):
        base = PollingThread(POLL_INTERVAL_S, "unimplemented")
        with self.assertRaises(NotImplementedError):
            base._tick()


if __name__ == "__main__":
    unittest.main()
