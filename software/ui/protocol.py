#!/usr/bin/env python3
"""
protocol.py — Shared wire format for ui_server / ui_client: connection
constants and length-prefixed MessagePack framing over a Unix Domain
Socket. Used by both sides so the framing logic exists in exactly one
place.
"""

import os
import socket
import struct
from typing import Optional

import msgpack

SOCKET_PATH = os.environ.get("UI_SOCKET_PATH", "/tmp/ui_server.sock")
DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64


def send_msg(sock: socket.socket, data: dict) -> None:
    """Send a length-prefixed MessagePack message."""
    payload = msgpack.packb(data, use_bin_type=True)
    header = struct.pack(">I", len(payload))
    sock.sendall(header + payload)


def recv_msg(sock: socket.socket) -> Optional[dict]:
    """Receive a length-prefixed MessagePack message. Returns None on disconnect."""
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
        try:
            chunk = sock.recv(n - len(buf))
        except OSError:
            return None
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)
