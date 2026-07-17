#!/usr/bin/env python3
"""Tests for default_ui.DefaultUI: state machine, button handling, and the
PREEMPTED / reconnect lifecycle. All collaborators (ui_client, application
manager, etc.) are injected as mocks so no real hardware or socket is used.
"""

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock, call, patch

sys.path.insert(0, str(Path(__file__).parent.parent))

from default_ui import DefaultUI, UIState, BACK  # noqa: E402
from application_manager import AppEntry  # noqa: E402
from melodies import APP_LAUNCH_MELODY, BUTTON_CLICK_MELODY  # noqa: E402


def _make_ui(**overrides) -> DefaultUI:
    app_manager = MagicMock()
    app_manager.names.return_value = []

    battery_monitor = MagicMock()
    battery_monitor.is_low = False
    battery_monitor.voltage = 7.4

    system_info = MagicMock()
    system_info.get_ip_address.return_value = "192.168.1.10"
    system_info.get_cpu_temp.return_value = 48.7
    system_info.get_cpu_freq_mhz.return_value = 1500

    defaults = dict(
        ui_client=MagicMock(),
        application_manager=app_manager,
        system_info=system_info,
        battery_monitor=battery_monitor,
        renderer=MagicMock(),
        system_controller=MagicMock(),
        discord_alert_monitor=MagicMock(),
    )
    defaults.update(overrides)
    return DefaultUI(**defaults)


class TestButtonEdgeDetection(unittest.TestCase):
    def test_fires_once_per_press_release_cycle(self):
        ui = _make_ui()
        calls = []
        ui._handle_button("left", "pressed", lambda: calls.append(1))
        ui._handle_button("left", "pressed", lambda: calls.append(1))
        ui._handle_button("left", "long_pressed", lambda: calls.append(1))
        self.assertEqual(len(calls), 1)

        ui._handle_button("left", "released", lambda: calls.append(1))
        ui._handle_button("left", "pressed", lambda: calls.append(1))
        self.assertEqual(len(calls), 2)

    def test_left_press_plays_its_configured_click_melody(self):
        ui = _make_ui()
        ui._handle_button("left", "pressed", lambda: None)
        ui._client.play.assert_called_once_with(BUTTON_CLICK_MELODY["left"])

    def test_right_press_plays_its_configured_click_melody(self):
        ui = _make_ui()
        ui._handle_button("right", "pressed", lambda: None)
        ui._client.play.assert_called_once_with(BUTTON_CLICK_MELODY["right"])

    def test_no_click_while_held_or_released(self):
        ui = _make_ui()
        ui._handle_button("left", "pressed", lambda: None)
        ui._client.play.reset_mock()
        ui._handle_button("left", "pressed", lambda: None)
        ui._handle_button("left", "long_pressed", lambda: None)
        ui._handle_button("left", "released", lambda: None)
        ui._client.play.assert_not_called()


class TestMainMenuNavigation(unittest.TestCase):
    def test_left_cycles_main_menu(self):
        ui = _make_ui()
        self.assertEqual(ui._main_menu.selected, "Applications")
        ui._on_left()
        self.assertEqual(ui._main_menu.selected, "System")
        ui._on_left()
        self.assertEqual(ui._main_menu.selected, "Applications")

    def test_select_applications_enters_application_menu(self):
        app_manager = MagicMock()
        app_manager.names.return_value = ["Maze", "Motor Test"]
        ui = _make_ui(application_manager=app_manager)

        ui._on_right()  # select "Applications"

        self.assertEqual(ui._state, UIState.APPLICATION_MENU)
        self.assertEqual(ui._app_menu.items, ["Maze", "Motor Test", BACK])

    def test_select_system_enters_system_menu(self):
        ui = _make_ui()
        ui._on_left()  # move to "System"
        ui._on_right()
        self.assertEqual(ui._state, UIState.SYSTEM_MENU)
        self.assertEqual(ui._system_menu.items, ["Reboot", "Shutdown", BACK])


class TestApplicationMenu(unittest.TestCase):
    def test_back_returns_to_main(self):
        app_manager = MagicMock()
        app_manager.names.return_value = ["Maze"]
        ui = _make_ui(application_manager=app_manager)
        ui._on_right()  # enter application menu
        ui._app_menu.next()  # Maze -> Back
        self.assertEqual(ui._app_menu.selected, BACK)

        ui._on_right()  # select Back

        self.assertEqual(ui._state, UIState.MAIN)

    def test_selecting_app_launches_it(self):
        entry = AppEntry(name="Maze", command=["python3", "maze.py"], priority=10)
        app_manager = MagicMock()
        app_manager.names.return_value = ["Maze"]
        app_manager.get.return_value = entry
        process = MagicMock()
        app_manager.launch.return_value = process
        app_manager.wait.return_value = 0

        ui = _make_ui(application_manager=app_manager)
        ui._on_right()  # enter application menu; selection = "Maze"

        ui._on_right()  # select "Maze" -> launch

        app_manager.launch.assert_called_once_with(entry)
        ui._client.play.assert_any_call(APP_LAUNCH_MELODY)
        ui._client.disconnect.assert_called_once()
        app_manager.wait.assert_called_once_with(process)
        self.assertEqual(ui._state, UIState.MAIN)
        self.assertFalse(ui._connected)

    def test_launch_notification_plays_before_disconnect(self):
        entry = AppEntry(name="Maze", command=["python3", "maze.py"], priority=10)
        app_manager = MagicMock()
        app_manager.names.return_value = ["Maze"]
        app_manager.get.return_value = entry
        app_manager.launch.return_value = MagicMock()
        app_manager.wait.return_value = 0

        ui = _make_ui(application_manager=app_manager)
        ui._on_right()  # enter application menu
        ui._on_right()  # select "Maze" -> launch

        client = ui._client
        launch_play_index = client.method_calls.index(call.play(APP_LAUNCH_MELODY))
        disconnect_index = client.method_calls.index(call.disconnect())
        self.assertLess(launch_play_index, disconnect_index)


class TestSystemMenuAndConfirm(unittest.TestCase):
    def _enter_system_menu(self, ui: DefaultUI) -> None:
        ui._on_left()
        ui._on_right()

    def test_reboot_requires_confirmation(self):
        ui = _make_ui()
        self._enter_system_menu(ui)
        self.assertEqual(ui._system_menu.selected, "Reboot")

        ui._on_right()

        self.assertEqual(ui._state, UIState.CONFIRM_REBOOT)
        ui._system_controller.reboot.assert_not_called()

    def test_confirm_reboot_yes_calls_controller(self):
        ui = _make_ui()
        self._enter_system_menu(ui)
        ui._on_right()  # -> CONFIRM_REBOOT
        ui._system_controller.reboot.return_value = True

        ui._on_right()  # Yes

        ui._system_controller.reboot.assert_called_once()

    def test_confirm_reboot_no_returns_to_system_menu(self):
        ui = _make_ui()
        self._enter_system_menu(ui)
        ui._on_right()  # -> CONFIRM_REBOOT

        ui._on_left()  # No

        self.assertEqual(ui._state, UIState.SYSTEM_MENU)

    def test_reboot_failure_falls_back_to_system_menu(self):
        ui = _make_ui()
        self._enter_system_menu(ui)
        ui._on_right()  # -> CONFIRM_REBOOT
        ui._system_controller.reboot.return_value = False

        ui._on_right()  # Yes, but it fails

        self.assertEqual(ui._state, UIState.SYSTEM_MENU)

    def test_system_menu_back_returns_to_main(self):
        ui = _make_ui()
        self._enter_system_menu(ui)
        ui._system_menu.next()
        ui._system_menu.next()
        self.assertEqual(ui._system_menu.selected, BACK)

        ui._on_right()

        self.assertEqual(ui._state, UIState.MAIN)


class TestPreemptedHandling(unittest.TestCase):
    def test_loop_once_propagates_connection_error_from_buttons(self):
        client = MagicMock()
        client.get_buttons.side_effect = ConnectionError("preempted")
        ui = _make_ui(ui_client=client)

        with self.assertRaises(ConnectionError):
            ui._loop_once()

    def test_ensure_connected_retries_until_success(self):
        client = MagicMock()
        client.connect.side_effect = [ConnectionError("no server"), None]
        ui = _make_ui(ui_client=client)

        with patch("default_ui.time.sleep"):
            ui._ensure_connected()

        self.assertTrue(ui._connected)
        self.assertEqual(client.connect.call_count, 2)

    def test_run_reconnects_after_preempted(self):
        client = MagicMock()
        client.get_buttons.return_value = {"left": "released", "right": "released"}

        call_count = {"n": 0}

        def fake_get_buttons():
            call_count["n"] += 1
            if call_count["n"] == 2:
                raise ConnectionError("preempted")
            if call_count["n"] >= 4:
                ui._running = False
            return {"left": "released", "right": "released"}

        client.get_buttons.side_effect = fake_get_buttons

        ui = _make_ui(ui_client=client)
        with patch("default_ui.time.sleep"):
            ui.run()

        # connect() called once for the initial connection and once more
        # after the simulated PREEMPTED disconnect.
        self.assertGreaterEqual(client.connect.call_count, 2)

    def test_run_recovers_from_a_raw_transport_error(self):
        """
        Regression test for a real bug found on hardware: unlike an explicit
        PREEMPTED response, a raw transport error (e.g. a broken pipe) does
        NOT clear ui_client's internal socket. If DefaultUI didn't force a
        disconnect() itself on any ConnectionError, the next connect() call
        would raise "Already connected" forever — an unrecoverable loop.
        This fake client reproduces that exact state-tracking behavior.
        """

        class _StatefulFakeClient:
            def __init__(self):
                self.sock_open = False
                self.play = MagicMock()
                self.display = MagicMock()
                self._get_buttons_calls = 0

            def connect(self, priority):
                if self.sock_open:
                    raise ConnectionError("Already connected")
                self.sock_open = True

            def disconnect(self):
                self.sock_open = False

            def get_buttons(self):
                self._get_buttons_calls += 1
                if self._get_buttons_calls == 2:
                    # Simulate a broken pipe: raises without clearing
                    # sock_open, unlike an explicit PREEMPTED response.
                    raise ConnectionError("[Errno 32] Broken pipe")
                if self._get_buttons_calls >= 4:
                    ui._running = False
                return {"left": "released", "right": "released"}

        client = _StatefulFakeClient()
        ui = _make_ui(ui_client=client)

        # signal.signal() only works in the main thread, so it must be
        # stubbed out here — that part of run() is covered separately by
        # TestSignalShutdown.
        with patch("default_ui.time.sleep"), patch("default_ui.signal.signal"):
            thread = threading.Thread(target=ui.run, daemon=True)
            thread.start()
            thread.join(timeout=2.0)

        self.assertFalse(
            thread.is_alive(),
            "run() never returned — stuck retrying connect() forever",
        )
        self.assertFalse(ui._running)


class TestSignalShutdown(unittest.TestCase):
    """A SIGTERM/SIGINT during a blocking ui_server call must unblock the
    call immediately (not hang until the next syscall auto-retries)."""

    def test_signal_during_blocking_call_breaks_the_loop_cleanly(self):
        client = MagicMock()

        def fake_get_buttons():
            # Simulate the signal arriving while blocked inside this call.
            ui._handle_signal(15, None)

        client.get_buttons.side_effect = fake_get_buttons
        ui = _make_ui(ui_client=client)

        with patch("default_ui.time.sleep"):
            ui.run()  # must return promptly, not hang

        self.assertFalse(ui._running)
        client.disconnect.assert_called_once()
        ui._battery.stop.assert_called_once()
        ui._discord_alerts.stop.assert_called_once()


class TestStatusRotationCaching(unittest.TestCase):
    """Status values (fetched via subprocess/I2C) must only be refreshed at
    the rotation boundary, not on every render frame."""

    def test_initial_update_populates_value_immediately(self):
        ui = _make_ui()
        ui._update_status_rotation()
        self.assertEqual(ui._status_value(), "7.40V")

    def test_no_refetch_within_the_same_rotation_window(self):
        system_info = MagicMock()
        system_info.get_cpu_temp.return_value = 48.7
        system_info.get_cpu_freq_mhz.return_value = 1500
        ui = _make_ui(system_info=system_info)

        with patch("default_ui.time.monotonic", return_value=1000.0):
            ui._update_status_rotation()  # initial fetch: battery
        with patch("default_ui.time.monotonic", return_value=1000.5):
            ui._update_status_rotation()  # still inside the 2s window

        system_info.get_cpu_temp.assert_not_called()
        system_info.get_cpu_freq_mhz.assert_not_called()

    def test_rotation_boundary_refreshes_and_advances(self):
        system_info = MagicMock()
        system_info.get_cpu_temp.return_value = 48.7
        ui = _make_ui(system_info=system_info)

        with patch("default_ui.time.monotonic", return_value=1000.0):
            ui._update_status_rotation()  # initial fetch: battery
        with patch("default_ui.time.monotonic", return_value=1002.5):
            ui._update_status_rotation()  # boundary crossed -> cpu_temp

        system_info.get_cpu_temp.assert_called_once()
        self.assertEqual(ui._status_value(), "48.7C")


if __name__ == "__main__":
    unittest.main(verbosity=2)
