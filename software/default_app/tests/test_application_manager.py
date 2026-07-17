#!/usr/bin/env python3
"""Tests for application_manager.ApplicationManager."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock

sys.path.insert(0, str(Path(__file__).parent.parent))

from application_manager import ApplicationManager, AppEntry  # noqa: E402


def _write_yaml(content: str) -> str:
    f = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    f.write(content)
    f.close()
    return f.name


class TestApplicationManagerLoading(unittest.TestCase):
    def test_missing_config_yields_empty_list(self):
        mgr = ApplicationManager("/nonexistent/path/applications.yaml")
        self.assertEqual(mgr.apps, [])

    def test_valid_config_is_loaded(self):
        path = _write_yaml(
            """
applications:
  - name: Maze
    command: [python3, /opt/robot/apps/maze.py]
    priority: 10
  - name: Motor Test
    command: [python3, /opt/robot/apps/motor_test.py]
    priority: 20
"""
        )
        try:
            mgr = ApplicationManager(path)
            self.assertEqual(mgr.names(), ["Maze", "Motor Test"])
            maze = mgr.get("Maze")
            self.assertIsNotNone(maze)
            self.assertEqual(maze.command, ["python3", "/opt/robot/apps/maze.py"])
            self.assertEqual(maze.priority, 10)
        finally:
            Path(path).unlink()

    def test_syntax_error_yields_empty_list(self):
        path = _write_yaml("applications: [this is: not valid: yaml")
        try:
            mgr = ApplicationManager(path)
            self.assertEqual(mgr.apps, [])
        finally:
            Path(path).unlink()

    def test_invalid_entry_is_skipped(self):
        path = _write_yaml(
            """
applications:
  - name: Good
    command: [python3, /opt/robot/apps/good.py]
    priority: 10
  - name: Missing Command
  - command: [python3, /opt/robot/apps/noname.py]
  - name: Bad Command Type
    command: "not-a-list"
"""
        )
        try:
            mgr = ApplicationManager(path)
            self.assertEqual(mgr.names(), ["Good"])
        finally:
            Path(path).unlink()

    def test_missing_priority_defaults(self):
        path = _write_yaml(
            """
applications:
  - name: NoPriority
    command: [python3, /opt/robot/apps/example.py]
"""
        )
        try:
            mgr = ApplicationManager(path)
            self.assertEqual(mgr.get("NoPriority").priority, 20)
        finally:
            Path(path).unlink()

    def test_get_unknown_name_returns_none(self):
        mgr = ApplicationManager("/nonexistent/path/applications.yaml")
        self.assertIsNone(mgr.get("Nope"))


class TestApplicationManagerLaunch(unittest.TestCase):
    def setUp(self):
        self.mgr = ApplicationManager("/nonexistent/path/applications.yaml")

    def test_launch_calls_popen_without_shell(self):
        entry = AppEntry(name="entry", command=["python3", "app.py"], priority=10)
        with patch("application_manager.subprocess.Popen") as popen:
            self.mgr.launch(entry)
            popen.assert_called_once_with(["python3", "app.py"], shell=False)

    def test_wait_returns_exit_code(self):
        process = MagicMock()
        process.wait.return_value = 0
        self.assertEqual(self.mgr.wait(process), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
