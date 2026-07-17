#!/usr/bin/env python3
"""
polling_thread.py — Base class for a component that runs a periodic
callback on its own background daemon thread.

Shared by BatteryMonitor and DiscordAlertMonitor, which otherwise each
re-implemented an identical start()/stop()/poll-loop.
"""

import threading
from typing import Optional


class PollingThread:
    """Runs self._tick() every poll_interval_s seconds on a daemon thread."""

    def __init__(self, poll_interval_s: float, thread_name: str) -> None:
        self._poll_interval_s = poll_interval_s
        self._thread_name = thread_name
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the background polling thread."""
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name=self._thread_name)
        self._thread.start()

    def stop(self) -> None:
        """Stop the background polling thread."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=self._poll_interval_s * 2)
            self._thread = None

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._tick()
            self._stop_event.wait(self._poll_interval_s)

    def _tick(self) -> None:
        """Called once per poll interval. Subclasses must override this."""
        raise NotImplementedError
