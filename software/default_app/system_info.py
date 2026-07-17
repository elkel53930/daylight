#!/usr/bin/env python3
"""
system_info.py — Reads Wi-Fi IP address, CPU temperature, and CPU frequency.

Every method returns None on failure instead of raising, so callers can
render "N/A" without special-casing exceptions.
"""

import fcntl
import logging
import re
import socket
import struct
import subprocess
from typing import Optional

logger = logging.getLogger("default_ui.system_info")

DEFAULT_INTERFACE = "wlan0"
DEFAULT_CPUFREQ_PATH = "/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq"

_SIOCGIFADDR = 0x8915
_TEMP_RE = re.compile(r"temp=([-\d.]+)")


class SystemInfo:
    """Collects Wi-Fi IP, CPU temperature, and CPU frequency."""

    def __init__(
        self,
        interface: str = DEFAULT_INTERFACE,
        cpufreq_path: str = DEFAULT_CPUFREQ_PATH,
    ) -> None:
        self._interface = interface
        self._cpufreq_path = cpufreq_path

    def get_ip_address(self) -> Optional[str]:
        """Return the IPv4 address of the configured interface, or None."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            packed_iface = struct.pack("256s", self._interface.encode("utf-8")[:15])
            raw = fcntl.ioctl(sock.fileno(), _SIOCGIFADDR, packed_iface)
            return socket.inet_ntoa(raw[20:24])
        except OSError as exc:
            logger.debug("Could not read IP for %s: %s", self._interface, exc)
            return None
        finally:
            sock.close()

    def get_cpu_temp(self) -> Optional[float]:
        """Return the CPU temperature in Celsius via `vcgencmd measure_temp`, or None."""
        try:
            output = subprocess.run(
                ["vcgencmd", "measure_temp"],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=True,
            ).stdout
        except (OSError, subprocess.SubprocessError) as exc:
            logger.warning("vcgencmd measure_temp failed: %s", exc)
            return None

        match = _TEMP_RE.search(output)
        if not match:
            logger.warning("Could not parse vcgencmd output: %r", output)
            return None
        try:
            return float(match.group(1))
        except ValueError:
            logger.warning("Could not convert temperature value: %r", match.group(1))
            return None

    def get_cpu_freq_mhz(self) -> Optional[int]:
        """
        Return the current CPU frequency in MHz, or None.

        cpuinfo_cur_freq is only root-readable on some kernels, so this
        shells out via sudo instead of opening the file directly (see the
        sudoers setup in README.md).
        """
        try:
            output = subprocess.run(
                ["sudo", "cat", self._cpufreq_path],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=True,
            ).stdout
        except (OSError, subprocess.SubprocessError) as exc:
            logger.warning("Could not read %s: %s", self._cpufreq_path, exc)
            return None

        try:
            value = int(output.strip())
        except ValueError:
            logger.warning("Unexpected cpufreq value: %r", output)
            return None

        # cpuinfo_cur_freq is reported in kHz on Linux; divide by 1000 for MHz.
        return value // 1000
