#!/usr/bin/env python3
"""
hysteresis.py — Edge-triggered threshold state with separate set/clear
points, so a reading oscillating near the threshold doesn't repeatedly
flip the state.

Shared by BatteryMonitor (voltage drops below a threshold) and
DiscordAlertMonitor (CPU temperature rises above a threshold), which
otherwise each hand-rolled the same set/clear comparison with mirrored
operators.
"""


class Hysteresis:
    """
    Tracks whether a value is in an "active" (alarm) state.

    direction="falls_below": active once value < threshold, clears once
        value >= clear_threshold (e.g. battery voltage going low).
    direction="rises_above": active once value > threshold, clears once
        value <= clear_threshold (e.g. CPU temperature going high).
    """

    def __init__(self, threshold: float, clear_threshold: float, direction: str) -> None:
        if direction not in ("falls_below", "rises_above"):
            raise ValueError(f"invalid direction: {direction!r}")
        self._threshold = threshold
        self._clear_threshold = clear_threshold
        self._direction = direction
        self.active = False

    def update(self, value: float) -> bool:
        """Update from a new reading and return the resulting active state."""
        if self._direction == "falls_below":
            if not self.active and value < self._threshold:
                self.active = True
            elif self.active and value >= self._clear_threshold:
                self.active = False
        else:
            if not self.active and value > self._threshold:
                self.active = True
            elif self.active and value <= self._clear_threshold:
                self.active = False
        return self.active
