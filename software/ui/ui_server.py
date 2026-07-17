#!/usr/bin/env python3
"""
UI Server for Raspberry Pi CM4
Manages SSD1331 OLED, tact switches (L/R), and PWM buzzer via Unix Domain Socket.
"""

import os
import sys
import errno
import select
import signal
import socket
import struct
import logging
import threading
import time
from pathlib import Path

import lgpio
import msgpack
from PIL import Image
from luma.core import cmdline as luma_cmdline

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SOCKET_PATH = os.environ.get("UI_SOCKET_PATH", "/tmp/ui_server.sock")
DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64

GPIO_BUTTON_LEFT = 26
GPIO_BUTTON_RIGHT = 19
GPIO_BUZZER = 13

DEBOUNCE_S = 0.020       # 20 ms
LONG_PRESS_S = 1.000     # 1000 ms

NOTE_FREQ = {
    'c': 524,  'd': 588,  'e': 660,  'f': 698,
    'g': 784,  'a': 880,  'b': 988,
    'C': 1048, 'D': 1176, 'E': 1320, 'F': 1396,
    'G': 1568, 'A': 1760, 'B': 1976,
}
NOTE_DURATION_S = 0.150  # 150 ms per note

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger("ui_server")

# ---------------------------------------------------------------------------
# MessagePack framing helpers
# ---------------------------------------------------------------------------

def send_msg(sock: socket.socket, data: dict) -> None:
    """Send a length-prefixed MessagePack message."""
    payload = msgpack.packb(data, use_bin_type=True)
    header = struct.pack(">I", len(payload))
    sock.sendall(header + payload)


def recv_msg(sock: socket.socket) -> dict | None:
    """Receive a length-prefixed MessagePack message. Returns None on disconnect."""
    header = _recv_exact(sock, 4)
    if header is None:
        return None
    length = struct.unpack(">I", header)[0]
    payload = _recv_exact(sock, length)
    if payload is None:
        return None
    return msgpack.unpackb(payload, raw=False)


def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except OSError:
            return None
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


# ---------------------------------------------------------------------------
# DisplayManager
# ---------------------------------------------------------------------------

class DisplayManager:
    """Manages the SSD1331 96x64 OLED display over SPI."""

    def __init__(self) -> None:
        self._device = self._init_device()
        self._lock = threading.Lock()

    def _init_device(self):
        try:
            parser = luma_cmdline.create_parser(description="UI Server Display")
            args = parser.parse_args([
                "--interface", "spi",
                "--display", "ssd1331",
                "--width", str(DISPLAY_WIDTH),
                "--height", str(DISPLAY_HEIGHT),
                "--rotate", "2",
            ])
            device = luma_cmdline.create_device(args)
            logger.info("Display initialised")
            return device
        except Exception as exc:
            logger.error("Display init failed: %s", exc)
            return None

    def show_splash(self) -> None:
        """Display splash.png if it exists, otherwise show black screen."""
        splash = Path(__file__).parent / "resources" / "splash.png"
        if splash.exists():
            try:
                img = Image.open(str(splash)).convert("RGB").resize(
                    (DISPLAY_WIDTH, DISPLAY_HEIGHT)
                )
                self.display(img)
                return
            except Exception as exc:
                logger.error("Splash load failed: %s", exc)
        self.clear()

    def display(self, image: Image.Image) -> None:
        """Display a PIL RGB image (96x64). Raises ValueError on bad input."""
        if image.mode != "RGB":
            raise ValueError("image must be RGB mode")
        if image.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
            raise ValueError(
                f"image must be {DISPLAY_WIDTH}x{DISPLAY_HEIGHT}, got {image.size}"
            )
        if self._device is None:
            logger.warning("Display unavailable, skipping render")
            return
        with self._lock:
            try:
                self._device.display(image)
            except Exception as exc:
                logger.error("Display error: %s", exc)

    def clear(self) -> None:
        """Fill the screen with black."""
        blank = Image.new("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), (0, 0, 0))
        if self._device is None:
            return
        with self._lock:
            try:
                self._device.display(blank)
            except Exception as exc:
                logger.error("Clear error: %s", exc)

    def cleanup(self) -> None:
        self.clear()
        if self._device is not None:
            try:
                self._device.cleanup()
            except Exception as exc:
                logger.error("Display cleanup error: %s", exc)


# ---------------------------------------------------------------------------
# ButtonManager
# ---------------------------------------------------------------------------

class ButtonManager:
    """
    Monitors GPIO 19 (LEFT) and GPIO 26 (RIGHT) with debounce and long-press.
    State is returned on demand; no event queue is maintained.
    """

    def __init__(self, gpio_handle: int) -> None:
        self._h = gpio_handle
        self._lock = threading.Lock()
        now = time.monotonic()
        # Per-pin state: raw level, last raw-change time, stable level, stable-since
        self._state: dict[int, dict] = {
            GPIO_BUTTON_LEFT: {
                "raw": 1, "raw_ts": now, "stable": 1, "stable_ts": now
            },
            GPIO_BUTTON_RIGHT: {
                "raw": 1, "raw_ts": now, "stable": 1, "stable_ts": now
            },
        }
        self._running = True
        self._init_gpio()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="btn-poll")
        self._thread.start()

    def _init_gpio(self) -> None:
        for pin in (GPIO_BUTTON_LEFT, GPIO_BUTTON_RIGHT):
            try:
                lgpio.gpio_claim_input(self._h, pin, lgpio.SET_PULL_UP)
            except Exception as exc:
                logger.error("Button GPIO init failed pin %d: %s", pin, exc)

    def _poll_loop(self) -> None:
        while self._running:
            now = time.monotonic()
            for pin in (GPIO_BUTTON_LEFT, GPIO_BUTTON_RIGHT):
                try:
                    val = lgpio.gpio_read(self._h, pin)
                except Exception as exc:
                    logger.error("Button read error pin %d: %s", pin, exc)
                    time.sleep(0.005)
                    continue
                with self._lock:
                    s = self._state[pin]
                    if val != s["raw"]:
                        s["raw"] = val
                        s["raw_ts"] = now
                    # Apply debounce: promote to stable after DEBOUNCE_S of stability
                    if val != s["stable"] and (now - s["raw_ts"]) >= DEBOUNCE_S:
                        s["stable"] = val
                        s["stable_ts"] = now
            time.sleep(0.005)

    def get_state(self) -> dict:
        """Return {left: state, right: state} where state ∈ {released, pressed, long_pressed}."""
        result = {}
        now = time.monotonic()
        with self._lock:
            for pin, name in ((GPIO_BUTTON_LEFT, "left"), (GPIO_BUTTON_RIGHT, "right")):
                s = self._state[pin]
                if s["stable"] == 1:  # pull-up: 1 = released
                    result[name] = "released"
                elif (now - s["stable_ts"]) >= LONG_PRESS_S:
                    result[name] = "long_pressed"
                else:
                    result[name] = "pressed"
        return result

    def cleanup(self) -> None:
        self._running = False
        self._thread.join(timeout=0.5)
        for pin in (GPIO_BUTTON_LEFT, GPIO_BUTTON_RIGHT):
            try:
                lgpio.gpio_free(self._h, pin)
            except Exception as exc:
                logger.error("Button GPIO free error pin %d: %s", pin, exc)


# ---------------------------------------------------------------------------
# BuzzerManager
# ---------------------------------------------------------------------------

class BuzzerManager:
    """
    Controls the PWM buzzer on GPIO 14.
    Melody playback runs in a background thread; new play() preempts ongoing melody.
    """

    def __init__(self, gpio_handle: int) -> None:
        self._h = gpio_handle
        self._melody_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._init_pwm()

    def _init_pwm(self) -> None:
        try:
            lgpio.gpio_claim_output(self._h, GPIO_BUZZER, 0, 0)
            lgpio.tx_pwm(self._h, GPIO_BUZZER, 100, 0)  # silent
        except Exception as exc:
            logger.error("Buzzer PWM init failed: %s", exc)

    def play(self, melody: str) -> None:
        """Start playing melody; preempts any ongoing melody."""
        with self._melody_lock:
            self._stop_event.set()
            if self._thread is not None:
                self._thread.join(timeout=NOTE_DURATION_S * 2)
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._play_melody, args=(melody,), daemon=True, name="buzzer"
            )
            self._thread.start()

    def _play_melody(self, melody: str) -> None:
        for ch in melody:
            if self._stop_event.is_set():
                break
            freq = NOTE_FREQ.get(ch)
            try:
                if freq:
                    lgpio.tx_pwm(self._h, GPIO_BUZZER, freq, 50)
                else:
                    lgpio.tx_pwm(self._h, GPIO_BUZZER, 100, 0)  # rest
            except Exception as exc:
                logger.error("Buzzer PWM error: %s", exc)
            self._stop_event.wait(NOTE_DURATION_S)
        # Silence after melody completes
        try:
            lgpio.tx_pwm(self._h, GPIO_BUZZER, 100, 0)
        except Exception as exc:
            logger.error("Buzzer stop error: %s", exc)

    def cleanup(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            lgpio.tx_pwm(self._h, GPIO_BUZZER, 100, 0)
            lgpio.gpio_free(self._h, GPIO_BUZZER)
        except Exception as exc:
            logger.error("Buzzer cleanup error: %s", exc)


# ---------------------------------------------------------------------------
# ClientManager
# ---------------------------------------------------------------------------

class ClientManager:
    """
    Manages the Unix Domain Socket server.
    Supports priority-based preemption; only one client is active at a time.
    Uses select() so new high-priority connections can preempt the current client.
    """

    def __init__(
        self,
        display_mgr: DisplayManager,
        button_mgr: ButtonManager,
        buzzer_mgr: BuzzerManager,
    ) -> None:
        self._display = display_mgr
        self._buttons = button_mgr
        self._buzzer = buzzer_mgr
        self._shutdown = False
        self._server_sock = self._create_server_socket()

    def _create_server_socket(self) -> socket.socket:
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(SOCKET_PATH)
        os.chmod(SOCKET_PATH, 0o666)
        sock.listen(5)
        sock.setblocking(False)
        logger.info("Listening on %s", SOCKET_PATH)
        return sock

    def run(self) -> None:
        """Main select loop: accept connections and dispatch commands."""
        current_conn: socket.socket | None = None
        current_priority: int | None = None

        while not self._shutdown:
            watch = [self._server_sock]
            if current_conn is not None:
                watch.append(current_conn)

            try:
                readable, _, exceptional = select.select(watch, [], watch, 1.0)
            except (OSError, ValueError):
                # Server socket closed during shutdown
                break

            for s in exceptional:
                if s is current_conn:
                    logger.warning("Client socket error, disconnecting")
                    current_conn.close()
                    current_conn = None
                    current_priority = None

            for s in readable:
                if s is self._server_sock:
                    # New incoming connection
                    try:
                        conn, _ = self._server_sock.accept()
                    except OSError:
                        continue

                    conn.settimeout(2.0)
                    try:
                        msg = recv_msg(conn)
                    except OSError:
                        conn.close()
                        continue
                    finally:
                        conn.settimeout(None)

                    if msg is None or msg.get("cmd") != "connect":
                        conn.close()
                        continue

                    new_priority = int(msg.get("priority", 999))

                    if current_conn is None:
                        # No active client — accept
                        current_conn = conn
                        current_priority = new_priority
                        logger.info("Client connected (priority=%d)", new_priority)
                    elif new_priority < current_priority:
                        # Higher-priority client: preempt existing
                        logger.info(
                            "Preempting priority=%d with priority=%d",
                            current_priority, new_priority,
                        )
                        try:
                            send_msg(current_conn, {"status": "PREEMPTED"})
                            current_conn.close()
                        except Exception:
                            pass
                        current_conn = conn
                        current_priority = new_priority
                        logger.info("Client connected (priority=%d)", new_priority)
                    else:
                        # Lower-priority (or equal) client: reject
                        logger.info(
                            "Rejecting lower-priority client (priority=%d, current=%d)",
                            new_priority, current_priority,
                        )
                        conn.close()

                elif s is current_conn:
                    msg = recv_msg(current_conn)
                    if msg is None:
                        logger.info("Client disconnected")
                        current_conn.close()
                        current_conn = None
                        current_priority = None
                    else:
                        self._dispatch(current_conn, msg)

    def _dispatch(self, conn: socket.socket, msg: dict) -> None:
        cmd = msg.get("cmd")
        try:
            if cmd == "display":
                raw = msg.get("image")
                if not isinstance(raw, (bytes, bytearray, memoryview)):
                    send_msg(conn, {"status": "error", "message": "invalid image data"})
                    return
                expected = DISPLAY_WIDTH * DISPLAY_HEIGHT * 3
                if len(raw) != expected:
                    send_msg(conn, {
                        "status": "error",
                        "message": f"image must be {expected} bytes, got {len(raw)}",
                    })
                    return
                img = Image.frombytes("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), bytes(raw))
                self._display.display(img)
                send_msg(conn, {"status": "ok"})

            elif cmd == "clear":
                self._display.clear()
                send_msg(conn, {"status": "ok"})

            elif cmd == "buttons":
                state = self._buttons.get_state()
                send_msg(conn, state)

            elif cmd == "play":
                melody = msg.get("melody", "")
                self._buzzer.play(str(melody))
                send_msg(conn, {"status": "ok"})

            else:
                send_msg(conn, {"status": "error", "message": f"unknown command: {cmd}"})

        except Exception as exc:
            logger.error("Dispatch error cmd=%s: %s", cmd, exc)
            try:
                send_msg(conn, {"status": "error", "message": str(exc)})
            except Exception:
                pass

    def cleanup(self) -> None:
        self._shutdown = True
        try:
            self._server_sock.close()
        except Exception:
            pass
        if os.path.exists(SOCKET_PATH):
            try:
                os.unlink(SOCKET_PATH)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# UIServer
# ---------------------------------------------------------------------------

class UIServer:
    """Top-level orchestrator: initialises hardware, handles signals, runs the main loop."""

    def __init__(self) -> None:
        self._h = lgpio.gpiochip_open(0)
        self._display = DisplayManager()
        self._buttons = ButtonManager(self._h)
        self._buzzer = BuzzerManager(self._h)
        self._client_mgr = ClientManager(self._display, self._buttons, self._buzzer)
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

    def _handle_signal(self, signum: int, _frame) -> None:
        logger.info("Received signal %d, shutting down", signum)
        self.cleanup()
        sys.exit(0)

    def run(self) -> None:
        self._display.show_splash()
        self._client_mgr.run()

    def cleanup(self) -> None:
        self._buzzer.cleanup()
        self._display.cleanup()
        self._client_mgr.cleanup()
        try:
            lgpio.gpiochip_close(self._h)
        except Exception as exc:
            logger.error("GPIO close error: %s", exc)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    server = UIServer()
    try:
        server.run()
    except Exception as exc:
        logger.critical("Unhandled exception: %s", exc, exc_info=True)
        server.cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()
