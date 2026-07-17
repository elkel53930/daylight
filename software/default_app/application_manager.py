#!/usr/bin/env python3
"""
application_manager.py — Loads the application list from YAML and launches
applications as child processes.

The YAML file is user/installer-editable, so no Default UI code changes are
needed to add a new application:

    applications:
      - name: Maze
        command: [python3, /opt/robot/apps/maze.py]
        priority: 10
"""

import logging
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import yaml

logger = logging.getLogger("default_ui.application_manager")

DEFAULT_CONFIG_PATH = "/etc/robot-ui/applications.yaml"


@dataclass(frozen=True)
class AppEntry:
    """A single launchable application."""

    name: str
    command: List[str]
    priority: int


class ApplicationManager:
    """Loads applications.yaml and launches/waits for child processes."""

    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH) -> None:
        self._config_path = config_path
        self._apps: List[AppEntry] = []
        self.reload()

    def reload(self) -> None:
        """Reload the application list from disk. Never raises."""
        path = Path(self._config_path)
        if not path.exists():
            logger.info("Applications config not found at %s; using empty list", path)
            self._apps = []
            return

        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            logger.error("Failed to parse %s: %s", path, exc)
            self._apps = []
            return
        except OSError as exc:
            logger.error("Failed to read %s: %s", path, exc)
            self._apps = []
            return

        self._apps = self._parse(data)

    @staticmethod
    def _parse(data: object) -> List[AppEntry]:
        if not isinstance(data, dict):
            logger.warning("applications.yaml root is not a mapping; using empty list")
            return []

        raw_apps = data.get("applications")
        if raw_apps is None:
            return []
        if not isinstance(raw_apps, list):
            logger.warning("'applications' is not a list; using empty list")
            return []

        apps: List[AppEntry] = []
        for entry in raw_apps:
            parsed = ApplicationManager._parse_entry(entry)
            if parsed is not None:
                apps.append(parsed)
        return apps

    @staticmethod
    def _parse_entry(entry: object) -> Optional[AppEntry]:
        if not isinstance(entry, dict):
            logger.warning("Skipping invalid application entry (not a mapping): %r", entry)
            return None

        name = entry.get("name")
        command = entry.get("command")
        priority = entry.get("priority", 20)

        if not isinstance(name, str) or not name:
            logger.warning("Skipping application entry with invalid name: %r", entry)
            return None
        if not isinstance(command, list) or not command or not all(
            isinstance(c, str) for c in command
        ):
            logger.warning("Skipping application '%s' with invalid command: %r", name, command)
            return None
        if not isinstance(priority, int):
            logger.warning(
                "Application '%s' has invalid priority %r; defaulting to 20", name, priority
            )
            priority = 20

        return AppEntry(name=name, command=list(command), priority=priority)

    @property
    def apps(self) -> List[AppEntry]:
        """Return all loaded application entries."""
        return list(self._apps)

    def names(self) -> List[str]:
        """Return the display names of all loaded applications."""
        return [app.name for app in self._apps]

    def get(self, name: str) -> Optional[AppEntry]:
        """Look up an application entry by name."""
        for app in self._apps:
            if app.name == name:
                return app
        return None

    def launch(self, entry: AppEntry) -> subprocess.Popen:
        """
        Start an application as a child process.
        Raises OSError if the executable cannot be started.
        """
        logger.info("Starting application '%s': %s", entry.name, entry.command)
        return subprocess.Popen(entry.command, shell=False)

    def wait(self, process: subprocess.Popen) -> int:
        """Block until the child process exits, returning its exit code."""
        return process.wait()
