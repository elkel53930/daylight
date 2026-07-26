#!/usr/bin/env python3
"""操縦PC(Ubuntu)側: ゲームコントローラ入力を機体へ送信する。

zeroconf で機体(remote_server.py)を検索し、TCP で接続してボタンの押下/
解放イベントとハートビートを送り続ける。接続が切れたら自動的に検索から
やり直す。ボタン配置・操作内容は software/manual_controller/README.md
および remote_controller.py の docstring を参照。

使い方(Ubuntu PC 上、DualSense を接続した状態で):
    python3 software/manual_controller/remote_client.py

必要な追加パッケージ: pygame, zeroconf
    pip install pygame zeroconf
(software/manual_controller/requirements.txt 参照)
"""

from __future__ import annotations

import argparse
import queue
import select
import socket
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import pygame

sys.path.insert(0, str(Path(__file__).parent))

import remote_protocol as proto
from input_mapping import (
    BUTTON_INDEX_TO_NAME,
    apply_rumble_messages,
    hat_to_events,
    process_incoming_lines,
)

DISCOVER_POLL_S = 0.5
LOOP_INTERVAL_S = 0.01


def discover(service_name: str, timeout_s: float) -> Optional[Tuple[str, int]]:
    """zeroconf で機体(service_name)を検索し (host, port) を返す。見つからなければ None。"""
    from zeroconf import ServiceBrowser, Zeroconf

    target_name = f"{service_name}.{proto.SERVICE_TYPE}"
    found: "queue.Queue" = queue.Queue()

    class Listener:
        def add_service(self, zc, type_, name):
            if name != target_name:
                return
            info = zc.get_service_info(type_, name)
            if info is not None:
                found.put(info)

        def update_service(self, zc, type_, name):
            pass

        def remove_service(self, zc, type_, name):
            pass

    zc = Zeroconf()
    browser = ServiceBrowser(zc, proto.SERVICE_TYPE, Listener())
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            try:
                info = found.get(timeout=DISCOVER_POLL_S)
            except queue.Empty:
                continue
            addrs = info.parsed_addresses()
            if addrs:
                return addrs[0], info.port
    finally:
        browser.cancel()
        zc.close()
    return None


def run_session(joystick: "pygame.joystick.Joystick", host: str, port: int, heartbeat_interval_s: float) -> None:
    """1接続分のメインループ。切断されたら例外(ConnectionError)を送出する。

    機体からの振動通知(rumble)を受け取り、コントローラを振動させる
    (1区間前進・90/180度旋回の完了合図。JOG系やL1/R1では送られてこない)。
    """
    with socket.create_connection((host, port), timeout=5.0) as sock:
        print(f"# 接続しました: {host}:{port}")
        prev_hat = (0, 0)
        last_heartbeat = time.monotonic()
        recv_buf = b""
        rumble_stop_at: Optional[float] = None
        try:
            while True:
                for event in pygame.event.get():
                    if event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                        name = BUTTON_INDEX_TO_NAME.get(event.button)
                        if name is not None:
                            action = (
                                proto.ACTION_DOWN
                                if event.type == pygame.JOYBUTTONDOWN
                                else proto.ACTION_UP
                            )
                            sock.sendall(proto.encode_button_event(name, action))
                    elif event.type == pygame.JOYHATMOTION:
                        if event.hat != 0:
                            continue
                        for name, action in hat_to_events(prev_hat, event.value):
                            sock.sendall(proto.encode_button_event(name, action))
                        prev_hat = event.value

                now = time.monotonic()
                if now - last_heartbeat >= heartbeat_interval_s:
                    sock.sendall(proto.encode_heartbeat())
                    last_heartbeat = now

                # 機体からの振動通知を非ブロッキングで確認する(既存の
                # sendall中心のループ速度に影響を与えないよう select で
                # 先にデータの有無だけ調べる)。
                readable, _, _ = select.select([sock], [], [], 0)
                if readable:
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("connection closed by robot")
                    recv_buf, messages = process_incoming_lines(recv_buf, chunk)
                    new_stop_at = apply_rumble_messages(joystick, messages, now)
                    if new_stop_at is not None:
                        rumble_stop_at = new_stop_at

                if rumble_stop_at is not None and now >= rumble_stop_at:
                    joystick.stop_rumble()
                    rumble_stop_at = None

                time.sleep(LOOP_INTERVAL_S)
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            raise ConnectionError(str(e)) from e


def main() -> int:
    ap = argparse.ArgumentParser(description="Daylight 遠隔操作クライアント(PC側)")
    ap.add_argument("--service-name", default="daylight", help="機体側の --service-name と合わせる")
    ap.add_argument("--discover-timeout", type=float, default=10.0, help="zeroconf検索のタイムアウト[秒]")
    ap.add_argument("--host", default=None, help="指定時はzeroconf検索をせず直接接続する")
    ap.add_argument("--port", type=int, default=proto.DEFAULT_CONTROL_PORT)
    ap.add_argument("--heartbeat-interval", type=float, default=0.2, help="ハートビート送信間隔[秒]")
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ゲームコントローラが見つかりません")
        return 1
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"コントローラ: {joystick.get_name()}")

    try:
        while True:
            if args.host is not None:
                host, port = args.host, args.port
            else:
                print(f"# zeroconfで機体を検索中(サービス名={args.service_name})...")
                found = discover(args.service_name, args.discover_timeout)
                if found is None:
                    print("# 見つかりませんでした。再検索します。")
                    continue
                host, port = found
                print(f"# 発見: {host}:{port}")

            try:
                run_session(joystick, host, port, args.heartbeat_interval)
            except ConnectionError as e:
                print(f"# 接続が切れました: {e}")
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n終了")
    finally:
        joystick.quit()
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
