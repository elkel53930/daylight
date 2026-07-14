#!/usr/bin/env python3
"""
UIClient — client library for ui_server.
Communicates over Unix Domain Socket using length-prefixed MessagePack frames.
"""

import socket
import struct
from typing import Optional

import msgpack
from PIL import Image

import os

SOCKET_PATH = os.environ.get("UI_SOCKET_PATH", "/tmp/ui_server.sock")
DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64


def _send_msg(sock: socket.socket, data: dict) -> None:
    payload = msgpack.packb(data, use_bin_type=True)
    header = struct.pack(">I", len(payload))
    sock.sendall(header + payload)


def _recv_msg(sock: socket.socket) -> Optional[dict]:
    header = _recv_exact(sock, 4)
    if header is None:
        return None
    length = struct.unpack(">I", header)[0]
    payload = _recv_exact(sock, length)
    if payload is None:
        return None
    return msgpack.unpackb(payload, raw=False)


def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


class UIClient:
    """
    Client for ui_server.

    Usage::

        client = UIClient()
        client.connect(priority=3)

        client.display(pil_image)
        client.clear()
        client.play("ccddeeff")

        buttons = client.get_buttons()
        # {"left": "released", "right": "pressed"}

        client.disconnect()
    """

    def __init__(self, socket_path: str = SOCKET_PATH) -> None:
        self._socket_path = socket_path
        self._sock: Optional[socket.socket] = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self, priority: int = 3) -> None:
        """
        Connect to ui_server with the given priority (0 = highest).
        Raises ConnectionError on failure.
        """
        if self._sock is not None:
            raise ConnectionError("Already connected")
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(self._socket_path)
        except OSError as exc:
            sock.close()
            raise ConnectionError(f"Cannot connect to {self._socket_path}: {exc}") from exc
        self._sock = sock
        _send_msg(self._sock, {"cmd": "connect", "priority": priority})

    def disconnect(self) -> None:
        """Disconnect from ui_server."""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _require_connected(self) -> None:
        if self._sock is None:
            raise ConnectionError("Not connected. Call connect() first.")

    def _check_preempted(self, response: Optional[dict]) -> None:
        """Raise ConnectionError if the server sent PREEMPTED."""
        if response is None:
            raise ConnectionError("Server closed the connection")
        if response.get("status") == "PREEMPTED":
            self._sock = None
            raise ConnectionError("Connection preempted by higher-priority client")

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def display(self, image: Image.Image) -> None:
        """
        Send a PIL RGB image (96x64) to the OLED display.
        Raises ValueError for wrong size/mode; ConnectionError on disconnect.
        """
        self._require_connected()
        if image.mode != "RGB":
            raise ValueError("image must be RGB mode")
        if image.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
            raise ValueError(
                f"image must be {DISPLAY_WIDTH}x{DISPLAY_HEIGHT}, got {image.size}"
            )
        raw = image.tobytes()
        _send_msg(self._sock, {
            "cmd": "display",
            "width": DISPLAY_WIDTH,
            "height": DISPLAY_HEIGHT,
            "image": raw,
        })
        resp = _recv_msg(self._sock)
        self._check_preempted(resp)

    def clear(self) -> None:
        """Clear the OLED to black."""
        self._require_connected()
        _send_msg(self._sock, {"cmd": "clear"})
        resp = _recv_msg(self._sock)
        self._check_preempted(resp)

    # ------------------------------------------------------------------
    # Buzzer
    # ------------------------------------------------------------------

    def play(self, melody: str) -> None:
        """
        Play a melody string (e.g. "ccddeeff").
        Characters outside [cdegfabCDEGFAB] are treated as rests.
        """
        self._require_connected()
        _send_msg(self._sock, {"cmd": "play", "melody": melody})
        resp = _recv_msg(self._sock)
        self._check_preempted(resp)

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------

    def get_buttons(self) -> dict:
        """
        Return current button states.

        Returns::

            {"left": "released", "right": "long_pressed"}

        Each value is one of: "released", "pressed", "long_pressed".
        """
        self._require_connected()
        _send_msg(self._sock, {"cmd": "buttons"})
        resp = _recv_msg(self._sock)
        self._check_preempted(resp)
        return resp

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "UIClient":
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()
