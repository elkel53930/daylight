#!/usr/bin/env python3
"""
test_protocol.py — Protocol and integration tests for ui_server / ui_client.

Hardware is mocked so these tests can run on any machine.
"""

import os
import signal
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import msgpack
from PIL import Image

# ---------------------------------------------------------------------------
# Patch hardware dependencies before importing server/client modules
# ---------------------------------------------------------------------------

# Mock lgpio
lgpio_mock = MagicMock()
lgpio_mock.SET_PULL_UP = 32
lgpio_mock.gpiochip_open.return_value = 0
lgpio_mock.gpio_read.return_value = 1          # default: button released
lgpio_mock.gpio_claim_input.return_value = 0
lgpio_mock.gpio_claim_output.return_value = 0
lgpio_mock.tx_pwm.return_value = 0
lgpio_mock.gpio_free.return_value = 0
lgpio_mock.gpiochip_close.return_value = 0
sys.modules["lgpio"] = lgpio_mock

# Mock luma modules
luma_device_mock = MagicMock()
luma_device_mock.width = 96
luma_device_mock.height = 64
luma_core_mock = MagicMock()
luma_core_mock.cmdline.create_parser.return_value = MagicMock()
luma_core_mock.cmdline.create_device.return_value = luma_device_mock
sys.modules["luma"] = MagicMock()
sys.modules["luma.core"] = luma_core_mock
sys.modules["luma.core.cmdline"] = luma_core_mock.cmdline
sys.modules["luma.oled"] = MagicMock()
sys.modules["luma.oled.device"] = MagicMock()

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from ui_server import (  # noqa: E402
    DISPLAY_HEIGHT,
    DISPLAY_WIDTH,
    ClientManager,
    BuzzerManager,
    ButtonManager,
    DisplayManager,
    UIServer,
    send_msg,
    recv_msg,
)
from ui_client import UIClient  # noqa: E402

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_rgb_image() -> Image.Image:
    return Image.new("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), (128, 64, 32))


def _raw_send(sock: socket.socket, data: dict) -> None:
    payload = msgpack.packb(data, use_bin_type=True)
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def _raw_recv(sock: socket.socket) -> dict:
    header = b""
    while len(header) < 4:
        header += sock.recv(4 - len(header))
    length = struct.unpack(">I", header)[0]
    payload = b""
    while len(payload) < length:
        payload += sock.recv(length - len(payload))
    return msgpack.unpackb(payload, raw=False)


def _start_server(socket_path: str) -> tuple:
    """Start a ClientManager server in a background thread. Returns (mgr, thread)."""
    display = MagicMock(spec=DisplayManager)
    buttons = MagicMock(spec=ButtonManager)
    buttons.get_state.return_value = {"left": "released", "right": "released"}
    buzzer = MagicMock(spec=BuzzerManager)

    mgr = ClientManager.__new__(ClientManager)
    mgr._display = display
    mgr._buttons = buttons
    mgr._buzzer = buzzer
    mgr._shutdown = False

    # Replace socket path
    if os.path.exists(socket_path):
        os.unlink(socket_path)
    srv_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_sock.bind(socket_path)
    os.chmod(socket_path, 0o666)
    srv_sock.listen(5)
    srv_sock.setblocking(False)
    mgr._server_sock = srv_sock

    t = threading.Thread(target=mgr.run, daemon=True)
    t.start()
    time.sleep(0.05)  # allow server to start
    return mgr, t, display, buttons, buzzer


def _connect_raw(socket_path: str, priority: int) -> socket.socket:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(socket_path)
    _raw_send(sock, {"cmd": "connect", "priority": priority})
    return sock


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestMessagePack(unittest.TestCase):
    """MessagePack framing helpers."""

    def test_roundtrip(self):
        r, w = socket.socketpair()
        try:
            send_msg(w, {"cmd": "clear"})
            msg = recv_msg(r)
            self.assertEqual(msg, {"cmd": "clear"})
        finally:
            r.close()
            w.close()

    def test_recv_returns_none_on_closed_socket(self):
        r, w = socket.socketpair()
        w.close()
        result = recv_msg(r)
        self.assertIsNone(result)
        r.close()

    def test_binary_image_roundtrip(self):
        r, w = socket.socketpair()
        raw = bytes(range(256)) * (DISPLAY_WIDTH * DISPLAY_HEIGHT * 3 // 256 + 1)
        raw = raw[: DISPLAY_WIDTH * DISPLAY_HEIGHT * 3]
        send_msg(w, {"cmd": "display", "image": raw})
        msg = recv_msg(r)
        self.assertEqual(msg["cmd"], "display")
        self.assertEqual(bytes(msg["image"]), raw)
        r.close()
        w.close()


class TestPriorityControl(unittest.TestCase):
    """Priority-based connection control."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_first_client_accepted(self):
        sock = _connect_raw(self._tmp, priority=3)
        # Send a clear command and expect ok
        _raw_send(sock, {"cmd": "clear"})
        resp = _raw_recv(sock)
        self.assertEqual(resp.get("status"), "ok")
        sock.close()

    def test_higher_priority_preempts_lower(self):
        low = _connect_raw(self._tmp, priority=5)
        time.sleep(0.05)

        # Connect with higher priority (lower number)
        high = _connect_raw(self._tmp, priority=1)
        time.sleep(0.05)

        # Low-priority client should receive PREEMPTED
        low.settimeout(1.0)
        preempted = _raw_recv(low)
        self.assertEqual(preempted.get("status"), "PREEMPTED")

        # High-priority client should be able to send commands
        _raw_send(high, {"cmd": "clear"})
        resp = _raw_recv(high)
        self.assertEqual(resp.get("status"), "ok")

        low.close()
        high.close()

    def test_lower_priority_rejected(self):
        high = _connect_raw(self._tmp, priority=1)
        time.sleep(0.05)

        # Connect with lower priority (higher number) — should be closed immediately
        low = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        low.connect(self._tmp)
        _raw_send(low, {"cmd": "connect", "priority": 9})
        low.settimeout(1.0)
        # Server closes the socket; recv returns empty bytes
        result = low.recv(16)
        self.assertEqual(result, b"")

        high.close()
        low.close()

    def test_equal_priority_rejected(self):
        first = _connect_raw(self._tmp, priority=3)
        time.sleep(0.05)

        second = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        second.connect(self._tmp)
        _raw_send(second, {"cmd": "connect", "priority": 3})
        second.settimeout(1.0)
        result = second.recv(16)
        self.assertEqual(result, b"")

        first.close()
        second.close()


class TestPreemptedMessage(unittest.TestCase):
    """PREEMPTED message format."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_preempted_message_content(self):
        victim = _connect_raw(self._tmp, priority=5)
        time.sleep(0.05)
        attacker = _connect_raw(self._tmp, priority=0)
        time.sleep(0.05)

        victim.settimeout(1.0)
        msg = _raw_recv(victim)
        self.assertIn("status", msg)
        self.assertEqual(msg["status"], "PREEMPTED")

        victim.close()
        attacker.close()


class TestButtonGet(unittest.TestCase):
    """Button state retrieval."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_buttons_released(self):
        self._buttons.get_state.return_value = {"left": "released", "right": "released"}
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {"cmd": "buttons"})
        resp = _raw_recv(sock)
        self.assertEqual(resp["left"], "released")
        self.assertEqual(resp["right"], "released")
        sock.close()

    def test_buttons_pressed(self):
        self._buttons.get_state.return_value = {"left": "pressed", "right": "long_pressed"}
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {"cmd": "buttons"})
        resp = _raw_recv(sock)
        self.assertEqual(resp["left"], "pressed")
        self.assertEqual(resp["right"], "long_pressed")
        sock.close()


class TestOLEDDisplay(unittest.TestCase):
    """OLED display command."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_display_ok(self):
        raw = bytes(DISPLAY_WIDTH * DISPLAY_HEIGHT * 3)
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {
            "cmd": "display",
            "width": DISPLAY_WIDTH,
            "height": DISPLAY_HEIGHT,
            "image": raw,
        })
        resp = _raw_recv(sock)
        self.assertEqual(resp.get("status"), "ok")
        self._display.display.assert_called_once()
        sock.close()

    def test_display_wrong_size_returns_error(self):
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {
            "cmd": "display",
            "width": DISPLAY_WIDTH,
            "height": DISPLAY_HEIGHT,
            "image": b"\x00" * 10,  # wrong size
        })
        resp = _raw_recv(sock)
        self.assertEqual(resp.get("status"), "error")
        sock.close()

    def test_clear_ok(self):
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {"cmd": "clear"})
        resp = _raw_recv(sock)
        self.assertEqual(resp.get("status"), "ok")
        self._display.clear.assert_called_once()
        sock.close()


class TestBuzzerPlay(unittest.TestCase):
    """Buzzer play command."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_play_ok(self):
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {"cmd": "play", "melody": "cdec"})
        resp = _raw_recv(sock)
        self.assertEqual(resp.get("status"), "ok")
        self._buzzer.play.assert_called_once_with("cdec")
        sock.close()

    def test_play_empty_melody(self):
        sock = _connect_raw(self._tmp, priority=3)
        _raw_send(sock, {"cmd": "play", "melody": ""})
        resp = _raw_recv(sock)
        self.assertEqual(resp.get("status"), "ok")
        sock.close()


class TestClientDisconnect(unittest.TestCase):
    """Client disconnect and server reconnect."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_server_accepts_new_client_after_disconnect(self):
        first = _connect_raw(self._tmp, priority=3)
        _raw_send(first, {"cmd": "clear"})
        _raw_recv(first)
        first.close()
        time.sleep(0.1)

        # Server should now accept a new client
        second = _connect_raw(self._tmp, priority=3)
        _raw_send(second, {"cmd": "clear"})
        resp = _raw_recv(second)
        self.assertEqual(resp.get("status"), "ok")
        second.close()


class TestServerReconnect(unittest.TestCase):
    """Server reconnect after client gone."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_multiple_reconnects(self):
        for _ in range(3):
            sock = _connect_raw(self._tmp, priority=3)
            _raw_send(sock, {"cmd": "clear"})
            resp = _raw_recv(sock)
            self.assertEqual(resp.get("status"), "ok")
            sock.close()
            time.sleep(0.05)


class TestSigtermHandling(unittest.TestCase):
    """SIGTERM causes server cleanup."""

    def test_sigterm_calls_cleanup(self):
        with (
            patch("lgpio.gpiochip_open", return_value=0),
            patch("lgpio.gpio_claim_input"),
            patch("lgpio.gpio_claim_output"),
            patch("lgpio.tx_pwm"),
            patch("lgpio.gpio_free"),
            patch("lgpio.gpiochip_close"),
            patch("lgpio.gpio_read", return_value=1),
        ):
            # Patch DisplayManager to avoid hardware
            with patch.object(DisplayManager, "_init_device", return_value=luma_device_mock):
                server = UIServer.__new__(UIServer)
                server._h = 0
                server._display = MagicMock(spec=DisplayManager)
                server._buttons = MagicMock(spec=ButtonManager)
                server._buzzer = MagicMock(spec=BuzzerManager)

                tmp = tempfile.mktemp(suffix=".sock")
                with patch("ui_server.SOCKET_PATH", tmp):
                    server._client_mgr = MagicMock(spec=ClientManager)

                # Register real signal handler and simulate SIGTERM
                import signal as _signal
                cleanup_called = []

                original_cleanup = server.cleanup

                def mock_cleanup():
                    cleanup_called.append(True)

                server.cleanup = mock_cleanup

                server._handle_signal(signal.SIGTERM, None)
                self.assertTrue(cleanup_called)


class TestUIClientAPI(unittest.TestCase):
    """UIClient high-level API against a real (mocked-hardware) server."""

    def setUp(self):
        self._tmp = tempfile.mktemp(suffix=".sock")
        self._mgr, self._thread, self._display, self._buttons, self._buzzer = (
            _start_server(self._tmp)
        )

    def tearDown(self):
        self._mgr.cleanup()
        if os.path.exists(self._tmp):
            os.unlink(self._tmp)

    def test_display_via_client(self):
        self._buttons.get_state.return_value = {"left": "released", "right": "released"}
        img = _make_rgb_image()
        client = UIClient(socket_path=self._tmp)
        client.connect(priority=3)
        client.display(img)
        client.disconnect()

    def test_clear_via_client(self):
        client = UIClient(socket_path=self._tmp)
        client.connect(priority=3)
        client.clear()
        client.disconnect()

    def test_play_via_client(self):
        client = UIClient(socket_path=self._tmp)
        client.connect(priority=3)
        client.play("cde")
        client.disconnect()

    def test_get_buttons_via_client(self):
        self._buttons.get_state.return_value = {"left": "pressed", "right": "released"}
        client = UIClient(socket_path=self._tmp)
        client.connect(priority=3)
        result = client.get_buttons()
        self.assertEqual(result["left"], "pressed")
        self.assertEqual(result["right"], "released")
        client.disconnect()

    def test_client_context_manager(self):
        with UIClient(socket_path=self._tmp) as client:
            client.connect(priority=3)
            client.clear()

    def test_not_connected_raises(self):
        client = UIClient(socket_path=self._tmp)
        with self.assertRaises(ConnectionError):
            client.clear()


if __name__ == "__main__":
    unittest.main(verbosity=2)
