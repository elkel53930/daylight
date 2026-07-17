#!/usr/bin/env python3
"""
battery.py — Reads battery voltage from an MCP3221 ADC over I2C and tracks
low-voltage state with hysteresis.

MCP3221 has no register map: a plain 2-byte read returns the latest
12-bit conversion. This uses the I2C_SLAVE ioctl directly on /dev/i2c-N so
no third-party I2C library is required.
"""

import fcntl
import logging
import os
import struct
import threading
from typing import Optional

from hysteresis import Hysteresis
from polling_thread import PollingThread

logger = logging.getLogger("default_ui.battery")

_I2C_SLAVE = 0x0703

DEFAULT_I2C_BUS = 1
DEFAULT_I2C_ADDRESS = 0x4D  # MCP3221 address; adjust to match actual wiring.
DEFAULT_POLL_INTERVAL_S = 2.0
DEFAULT_LOW_VOLTAGE_THRESHOLD = 6.5
DEFAULT_LOW_VOLTAGE_CLEAR_THRESHOLD = 6.7

ADC_MAX = 4095
ADC_VDD = 3.3
VOLTAGE_DIVIDER_RATIO = 11.0


class MCP3221Reader:
    """Reads a raw 12-bit sample from an MCP3221 ADC over I2C."""

    def __init__(self, address: int = DEFAULT_I2C_ADDRESS, bus: int = DEFAULT_I2C_BUS) -> None:
        self._address = address
        self._bus_path = f"/dev/i2c-{bus}"

    def read_raw(self) -> int:
        """Return the raw 12-bit ADC value. Raises OSError on I2C failure."""
        fd = os.open(self._bus_path, os.O_RDWR)
        try:
            fcntl.ioctl(fd, _I2C_SLAVE, self._address)
            data = os.read(fd, 2)
        finally:
            os.close(fd)
        return struct.unpack(">H", data)[0] & 0x0FFF


class BatteryMonitor(PollingThread):
    """
    Periodically samples the battery voltage in a background thread and
    exposes the latest reading plus a hysteresis-based low-voltage flag.
    """

    def __init__(
        self,
        reader: Optional[MCP3221Reader] = None,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
        low_voltage_threshold: float = DEFAULT_LOW_VOLTAGE_THRESHOLD,
        low_voltage_clear_threshold: float = DEFAULT_LOW_VOLTAGE_CLEAR_THRESHOLD,
    ) -> None:
        super().__init__(poll_interval_s, thread_name="battery-monitor")
        self._reader = reader or MCP3221Reader()
        self._low_state = Hysteresis(low_voltage_threshold, low_voltage_clear_threshold, "falls_below")

        self._lock = threading.Lock()
        self._voltage: Optional[float] = None

    @staticmethod
    def raw_to_voltage(raw: int) -> float:
        """Convert a raw 12-bit ADC value to battery voltage."""
        return raw / ADC_MAX * ADC_VDD * VOLTAGE_DIVIDER_RATIO

    def poll_once(self) -> Optional[float]:
        """Read the ADC once, update internal state, and return the voltage (or None)."""
        try:
            raw = self._reader.read_raw()
        except OSError as exc:
            logger.warning("Battery ADC read failed: %s", exc)
            return None

        voltage = self.raw_to_voltage(raw)
        with self._lock:
            self._voltage = voltage
            self._update_low_state(voltage)
        return voltage

    def _update_low_state(self, voltage: float) -> None:
        was_low = self._low_state.active
        is_low = self._low_state.update(voltage)
        if is_low and not was_low:
            logger.warning("Battery voltage low: %.2fV", voltage)
        elif was_low and not is_low:
            logger.info("Battery voltage recovered: %.2fV", voltage)

    @property
    def voltage(self) -> Optional[float]:
        """Return the most recently measured voltage, or None if never read."""
        with self._lock:
            return self._voltage

    @property
    def is_low(self) -> bool:
        """Return True if the battery is currently in the low-voltage state."""
        with self._lock:
            return self._low_state.active

    def _tick(self) -> None:
        self.poll_once()
