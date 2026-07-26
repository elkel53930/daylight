"""remote_server.py のネットワーク層(TCP受信・watchdog・切断処理)の統合テスト。

実ハードウェア(mob シリアル・zeroconf)は使わず、FakeBase を注入した
RemoteController + ローカル TCP ループバックで検証する。remote_server.py は
zeroconf import を register_zeroconf() 関数内に遅延させているため、
zeroconf 未導入でもこのファイルの import 自体は成功する
(register_zeroconf() 自体は呼ばない)。
"""

import queue
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import remote_protocol as proto
from remote_controller import RemoteController
import remote_server


class FakeBase:
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()

    def _record(self, name, *args):
        with self.lock:
            self.calls.append((name,) + args)

    def stop_at(self, speed_mmps, accel_mmps2, distance_mm):
        self._record("stop_at", speed_mmps, accel_mmps2, distance_mm)

    def turn(self, angle_rad):
        self._record("turn", angle_rad)

    def latch_forward(self):
        self._record("latch_forward")

    def latch_backward(self):
        self._record("latch_backward")

    def latch_turn_left(self):
        self._record("latch_turn_left")

    def latch_turn_right(self):
        self._record("latch_turn_right")

    def latch_stop(self):
        self._record("latch_stop")

    def emergency_stop(self):
        self._record("emergency_stop")

    def calls_snapshot(self):
        with self.lock:
            return list(self.calls)


def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class TestHandleClient(unittest.TestCase):
    def _start_server(self, controller, link_lost):
        port = free_tcp_port()
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", port))
        server_sock.listen(1)

        accepted = []

        def accept_once():
            conn, _addr = server_sock.accept()
            accepted.append(conn)
            remote_server.handle_client(conn, controller, link_lost)
            conn.close()

        t = threading.Thread(target=accept_once, daemon=True)
        t.start()
        return server_sock, port, t

    def test_button_event_reaches_controller(self):
        base = FakeBase()
        controller = RemoteController(base)
        link_lost = threading.Event()
        server_sock, port, server_thread = self._start_server(controller, link_lost)
        try:
            client = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            client.sendall(proto.encode_button_event(proto.DPAD_RIGHT, proto.ACTION_DOWN))
            time.sleep(0.1)
            controller.step()
            self.assertEqual(base.calls_snapshot(), [("turn", -1.5707963267948966)])
            client.close()
            server_thread.join(timeout=2.0)
            self.assertTrue(link_lost.is_set())
        finally:
            server_sock.close()

    def test_clean_disconnect_emergency_stops(self):
        base = FakeBase()
        controller = RemoteController(base)
        link_lost = threading.Event()
        server_sock, port, server_thread = self._start_server(controller, link_lost)
        try:
            client = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            client.sendall(proto.encode_heartbeat())
            time.sleep(0.05)
            client.close()
            server_thread.join(timeout=2.0)
            self.assertIn("emergency_stop", [c[0] for c in base.calls_snapshot()])
            self.assertTrue(link_lost.is_set())
        finally:
            server_sock.close()

    def test_watchdog_timeout_without_heartbeat(self):
        # WATCHDOG_TIMEOUT_S を短く差し替えてタイムアウトを高速に発生させる
        original_timeout = remote_server.WATCHDOG_TIMEOUT_S
        remote_server.WATCHDOG_TIMEOUT_S = 0.15
        try:
            base = FakeBase()
            controller = RemoteController(base)
            link_lost = threading.Event()
            server_sock, port, server_thread = self._start_server(controller, link_lost)
            try:
                client = socket.create_connection(("127.0.0.1", port), timeout=2.0)
                # 何も送らず放置 → watchdogタイムアウトでサーバー側から緊急停止するはず
                server_thread.join(timeout=2.0)
                self.assertFalse(server_thread.is_alive())
                self.assertIn("emergency_stop", [c[0] for c in base.calls_snapshot()])
                self.assertTrue(link_lost.is_set())
                client.close()
            finally:
                server_sock.close()
        finally:
            remote_server.WATCHDOG_TIMEOUT_S = original_timeout

    def test_should_stop_event_ends_client_loop(self):
        # メニューからの起動時、OLEDのLボタンでこのイベントが立つ想定
        # (remote_server.ui_loop 参照)。接続中でも即座に抜けられること。
        base = FakeBase()
        controller = RemoteController(base)
        link_lost = threading.Event()
        should_stop = threading.Event()
        port = free_tcp_port()
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", port))
        server_sock.listen(1)

        def accept_once():
            conn, _addr = server_sock.accept()
            remote_server.handle_client(conn, controller, link_lost, should_stop)
            conn.close()

        t = threading.Thread(target=accept_once, daemon=True)
        t.start()
        try:
            client = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            time.sleep(0.05)
            should_stop.set()
            t.join(timeout=2.0)
            self.assertFalse(t.is_alive())
            self.assertIn("emergency_stop", [c[0] for c in base.calls_snapshot()])
            client.close()
        finally:
            server_sock.close()

    def test_rumble_notification_is_sent_to_client(self):
        base = FakeBase()
        rumble_queue: "queue.Queue" = queue.Queue()
        controller = RemoteController(
            base, on_command_done=lambda: rumble_queue.put(100)
        )
        link_lost = threading.Event()
        port = free_tcp_port()
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", port))
        server_sock.listen(1)

        def accept_once():
            conn, _addr = server_sock.accept()
            remote_server.handle_client(conn, controller, link_lost, rumble_queue=rumble_queue)
            conn.close()

        t = threading.Thread(target=accept_once, daemon=True)
        t.start()
        try:
            client = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            client.sendall(proto.encode_button_event(proto.DPAD_RIGHT, proto.ACTION_DOWN))
            time.sleep(0.1)
            controller.step()  # turn成功 → on_command_done → rumble_queueに積まれる

            client.settimeout(2.0)
            raw = client.recv(4096)
            decoded = proto.decode_line(raw.decode("utf-8"))
            self.assertEqual(decoded, {"type": "rumble", "duration_ms": 100})

            client.close()
            t.join(timeout=2.0)
        finally:
            server_sock.close()

    def test_garbage_line_is_ignored_not_fatal(self):
        base = FakeBase()
        controller = RemoteController(base)
        link_lost = threading.Event()
        server_sock, port, server_thread = self._start_server(controller, link_lost)
        try:
            client = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            client.sendall(b"not json\n")
            client.sendall(proto.encode_button_event(proto.DPAD_LEFT, proto.ACTION_DOWN))
            time.sleep(0.1)
            controller.step()
            self.assertEqual(base.calls_snapshot(), [("turn", 1.5707963267948966)])
            client.close()
            server_thread.join(timeout=2.0)
        finally:
            server_sock.close()


if __name__ == "__main__":
    unittest.main()
