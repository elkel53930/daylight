#!/usr/bin/env python3
"""
system_controller.py — Executes system-level reboot and shutdown operations.
"""

import logging
import subprocess
from typing import List

logger = logging.getLogger("default_ui.system_controller")


class SystemController:
    """Issues reboot/shutdown requests to systemd via sudo."""

    def reboot(self) -> bool:
        """Request a system reboot. Returns True on success."""
        return self._run(["sudo", "systemctl", "reboot"], "reboot")

    def shutdown(self) -> bool:
        """Request a system shutdown. Returns True on success."""
        return self._run(["sudo", "systemctl", "poweroff"], "shutdown")

    @staticmethod
    def _run(command: List[str], action: str) -> bool:
        logger.info("Requesting %s: %s", action, command)
        try:
            subprocess.run(command, check=True, timeout=10.0)
            return True
        except (OSError, subprocess.SubprocessError) as exc:
            logger.error("%s failed: %s", action.capitalize(), exc)
            return False
