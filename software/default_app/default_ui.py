#!/usr/bin/env python3
"""
default_ui.py — Robot Default UI: the "home screen" / shell application.

Owns the ui_server connection at priority=100 (lowest priority). Shows
system status, lets the user launch other applications, and handles
reboot/shutdown. Treats PREEMPTED as a normal lifecycle event rather than
an error, and reconnects automatically once the UI becomes available again.
"""

import logging
import signal
import sys
import time
from enum import Enum, auto
from pathlib import Path
from typing import Dict, Optional

try:
    from ui_client import UIClient
except ImportError:  # pragma: no cover - fallback for repo-local development
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ui"))
    from ui_client import UIClient

from application_manager import ApplicationManager, DEFAULT_CONFIG_PATH
from battery import BatteryMonitor
from melodies import APP_LAUNCH_MELODY, BUTTON_CLICK_MELODY, LOW_BATTERY_MELODY
from menu import MenuManager
from renderer import UIRenderer
from system_controller import SystemController
from system_info import SystemInfo

logger = logging.getLogger("default_ui")

PRIORITY = 100
BUTTON_POLL_INTERVAL_S = 0.1
RECONNECT_INTERVAL_S = 1.0
STATUS_CYCLE_INTERVAL_S = 2.0
LOW_BATTERY_WARNING_INTERVAL_S = 5.0

STATUS_SEQUENCE = ("battery", "cpu_temp", "cpu_freq")


class UIState(Enum):
    MAIN = auto()
    APPLICATION_MENU = auto()
    SYSTEM_MENU = auto()
    CONFIRM_REBOOT = auto()
    CONFIRM_SHUTDOWN = auto()


class _ShutdownRequested(Exception):
    """
    Raised from the SIGTERM/SIGINT handler.

    A handler that merely sets a flag and returns normally lets CPython
    auto-restart the interrupted syscall (PEP 475), so a blocking
    ui_server socket call (e.g. get_buttons()) would never unblock. Raising
    here instead forces that call to return immediately with this
    exception, the same mechanism the default SIGINT handler uses to turn
    Ctrl+C into a KeyboardInterrupt.
    """


BACK = "Back"


class DefaultUI:
    """Top-level orchestrator: state machine, connection lifecycle, and main loop."""

    def __init__(
        self,
        ui_client: Optional[UIClient] = None,
        application_manager: Optional[ApplicationManager] = None,
        system_info: Optional[SystemInfo] = None,
        battery_monitor: Optional[BatteryMonitor] = None,
        renderer: Optional[UIRenderer] = None,
        system_controller: Optional[SystemController] = None,
        apps_config_path: str = DEFAULT_CONFIG_PATH,
    ) -> None:
        self._client = ui_client or UIClient()
        self._app_manager = application_manager or ApplicationManager(apps_config_path)
        self._system_info = system_info or SystemInfo()
        self._battery = battery_monitor or BatteryMonitor()
        self._renderer = renderer or UIRenderer()
        self._system_controller = system_controller or SystemController()

        self._main_menu = MenuManager(["Applications", "System"])
        self._app_menu = MenuManager([])
        self._system_menu = MenuManager([])

        self._state = UIState.MAIN
        self._connected = False
        self._running = True

        self._status_index = 0
        self._status_value_cache = "N/A"
        self._status_initialized = False
        self._last_status_switch = time.monotonic()
        self._last_low_battery_warning = 0.0

        self._button_prev: Dict[str, str] = {"left": "released", "right": "released"}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start background monitors and run the main loop until stopped."""
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

        self._battery.start()
        try:
            while self._running:
                try:
                    self._ensure_connected()
                    if not self._running:
                        break
                    self._loop_once()
                except ConnectionError as exc:
                    logger.info("UI connection lost (%s); will reconnect", exc)
                    # ui_client only clears its internal socket on an explicit
                    # PREEMPTED message; a raw transport error (e.g. a broken
                    # pipe) leaves it set, which would make the next
                    # connect() call raise "Already connected" forever. Force
                    # a clean slate here regardless of which case this was.
                    self._client.disconnect()
                    self._connected = False
                except _ShutdownRequested:
                    break
        finally:
            self._shutdown()

    def _handle_signal(self, signum: int, _frame) -> None:
        logger.info("Received signal %d, stopping", signum)
        self._running = False
        raise _ShutdownRequested()

    def _shutdown(self) -> None:
        self._battery.stop()
        if self._connected:
            self._client.disconnect()
            self._connected = False

    def _ensure_connected(self) -> None:
        while self._running and not self._connected:
            try:
                self._client.connect(priority=PRIORITY)
                self._connected = True
                self._state = UIState.MAIN
                self._main_menu.reset()
                logger.info("Connected to ui_server (priority=%d)", PRIORITY)
            except ConnectionError as exc:
                logger.warning("Could not connect to ui_server: %s; retrying", exc)
                time.sleep(RECONNECT_INTERVAL_S)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def _loop_once(self) -> None:
        self._poll_buttons()
        self._update_status_rotation()
        self._maybe_play_low_battery_warning()
        self._render()
        time.sleep(BUTTON_POLL_INTERVAL_S)

    def _update_status_rotation(self) -> None:
        # Status values (esp. CPU temp/freq) are fetched via subprocess, so
        # only refresh them at the rotation boundary rather than every
        # render frame.
        if not self._status_initialized:
            self._status_initialized = True
            self._last_status_switch = time.monotonic()
            self._refresh_status_value()
            return

        now = time.monotonic()
        if now - self._last_status_switch >= STATUS_CYCLE_INTERVAL_S:
            self._status_index = (self._status_index + 1) % len(STATUS_SEQUENCE)
            self._last_status_switch = now
            self._refresh_status_value()

    def _refresh_status_value(self) -> None:
        kind = STATUS_SEQUENCE[self._status_index]
        if kind == "battery":
            voltage = self._battery.voltage
            self._status_value_cache = f"{voltage:.2f}V" if voltage is not None else "N/A"
        elif kind == "cpu_temp":
            temp = self._system_info.get_cpu_temp()
            self._status_value_cache = f"{temp:.1f}C" if temp is not None else "N/A"
        elif kind == "cpu_freq":
            freq = self._system_info.get_cpu_freq_mhz()
            self._status_value_cache = f"{freq}MHz" if freq is not None else "N/A"

    def _maybe_play_low_battery_warning(self) -> None:
        if not self._battery.is_low:
            return
        now = time.monotonic()
        if now - self._last_low_battery_warning >= LOW_BATTERY_WARNING_INTERVAL_S:
            self._last_low_battery_warning = now
            self._client.play(LOW_BATTERY_MELODY)

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------

    def _poll_buttons(self) -> None:
        buttons = self._client.get_buttons()
        self._handle_button("left", buttons.get("left", "released"), self._on_left)
        self._handle_button("right", buttons.get("right", "released"), self._on_right)

    def _handle_button(self, name: str, state: str, on_press) -> None:
        prev = self._button_prev[name]
        if state == "released":
            self._button_prev[name] = "released"
            return
        self._button_prev[name] = state
        if prev == "released":
            self._client.play(BUTTON_CLICK_MELODY[name])
            on_press()

    def _on_left(self) -> None:
        if self._state == UIState.MAIN:
            self._main_menu.next()
        elif self._state == UIState.APPLICATION_MENU:
            self._app_menu.next()
        elif self._state == UIState.SYSTEM_MENU:
            self._system_menu.next()
        elif self._state in (UIState.CONFIRM_REBOOT, UIState.CONFIRM_SHUTDOWN):
            self._state = UIState.SYSTEM_MENU

    def _on_right(self) -> None:
        if self._state == UIState.MAIN:
            self._select_main_menu()
        elif self._state == UIState.APPLICATION_MENU:
            self._select_application_menu()
        elif self._state == UIState.SYSTEM_MENU:
            self._select_system_menu()
        elif self._state == UIState.CONFIRM_REBOOT:
            if not self._system_controller.reboot():
                self._state = UIState.SYSTEM_MENU
        elif self._state == UIState.CONFIRM_SHUTDOWN:
            if not self._system_controller.shutdown():
                self._state = UIState.SYSTEM_MENU

    def _select_main_menu(self) -> None:
        selected = self._main_menu.selected
        if selected == "Applications":
            self._app_manager.reload()
            self._app_menu.set_items(self._app_manager.names() + [BACK])
            self._state = UIState.APPLICATION_MENU
        elif selected == "System":
            self._system_menu.set_items(["Reboot", "Shutdown", BACK])
            self._state = UIState.SYSTEM_MENU

    def _select_application_menu(self) -> None:
        name = self._app_menu.selected
        if name is None or name == BACK:
            self._state = UIState.MAIN
            self._main_menu.reset()
            return
        entry = self._app_manager.get(name)
        if entry is not None:
            self._launch_app(entry)

    def _select_system_menu(self) -> None:
        selected = self._system_menu.selected
        if selected == "Reboot":
            self._state = UIState.CONFIRM_REBOOT
        elif selected == "Shutdown":
            self._state = UIState.CONFIRM_SHUTDOWN
        elif selected == BACK or selected is None:
            self._state = UIState.MAIN
            self._main_menu.reset()

    # ------------------------------------------------------------------
    # Application launch
    # ------------------------------------------------------------------

    def _launch_app(self, entry) -> None:
        try:
            process = self._app_manager.launch(entry)
        except OSError as exc:
            logger.error("Failed to launch '%s': %s", entry.name, exc)
            return

        # The melody keeps playing in ui_server's own buzzer thread even
        # after we disconnect below, so it isn't cut short.
        self._client.play(APP_LAUNCH_MELODY)
        self._client.disconnect()
        self._connected = False
        logger.info("Waiting for '%s' (pid=%s) to exit", entry.name, process.pid)
        self._app_manager.wait(process)
        logger.info("Application '%s' exited", entry.name)

        self._state = UIState.MAIN
        self._main_menu.reset()
        self._app_menu.reset()

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def _render(self) -> None:
        image = self._build_image()
        self._client.display(image)

    def _build_image(self):
        if self._state == UIState.MAIN:
            return self._renderer.render_main(
                ip_text=self._format_ip(),
                status_label=self._status_label(),
                status_value=self._status_value(),
                menu_items=self._main_menu.items,
                selected_index=self._main_menu.index,
                low_battery=self._battery.is_low,
            )
        if self._state == UIState.APPLICATION_MENU:
            return self._renderer.render_menu(
                "Applications", self._app_menu.items, self._app_menu.index
            )
        if self._state == UIState.SYSTEM_MENU:
            return self._renderer.render_menu(
                "System", self._system_menu.items, self._system_menu.index
            )
        if self._state == UIState.CONFIRM_REBOOT:
            return self._renderer.render_confirm("Reboot?")
        if self._state == UIState.CONFIRM_SHUTDOWN:
            return self._renderer.render_confirm("Shutdown?")
        return self._renderer.render_main(
            ip_text=self._format_ip(),
            status_label=self._status_label(),
            status_value=self._status_value(),
            menu_items=self._main_menu.items,
            selected_index=self._main_menu.index,
            low_battery=self._battery.is_low,
        )

    def _format_ip(self) -> str:
        ip = self._system_info.get_ip_address()
        return ip if ip else "N/A"

    def _status_label(self) -> str:
        return {
            "battery": "BAT",
            "cpu_temp": "CPU",
            "cpu_freq": "FREQ",
        }[STATUS_SEQUENCE[self._status_index]]

    def _status_value(self) -> str:
        return self._status_value_cache


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )
    app = DefaultUI()
    app.run()


if __name__ == "__main__":
    main()
