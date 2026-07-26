#!/usr/bin/env python3
"""機体(ラズパイ)側: 遠隔操作サーバー。

zeroconf で自身を広告し(software/manual_controller/README.md 参照)、
PC 側(remote_client.py)からの TCP 接続を受け付けてボタンイベントを
RemoteController に渡す。実際のボタン→機体動作の変換ロジックは
remote_controller.py(ハード非依存、ユニットテスト済み)が持つ。

安全設計:
    - 受信ソケットに短いタイムアウトを設定し、一定時間(WATCHDOG_TIMEOUT_S)
      メッセージ(ボタンイベント or ハートビート)が来なければリンク切断と
      みなして緊急停止する。PC 側は実入力が無くても一定間隔でハートビートを
      送るので、正常時に誤検知することはない。
    - MobileBase の abort_check をリンク切断フラグに直結しているため、
      stop_at()/turn() 等のブロッキング待ち中でも切断を即検知して中断できる
      (micromouse の中断ボタンと同じ仕組み)。
    - 切断時・起動終了時は必ず emergency_stop()(QSTP→ダメならMOT,0,0)する。

ui_server が起動していれば OLED に状態(待機中/接続中)を表示し、Lボタンで
終了できる(default_app のメニューから起動した場合、default_ui は子プロセスの
終了を待つだけで強制終了しないため、これが無いとメニューに戻れなくなる)。
ui_server が無い環境(SSH手動実行等)では自動的にスキップされ、Ctrl+C で
終了する。

使い方:
    software/venv/bin/python3 software/manual_controller/remote_server.py

必要な追加パッケージ: zeroconf(software/manual_controller/README.md 参照。
pip install 等の環境変更はユーザーが行うこと)。
"""

from __future__ import annotations

import argparse
import queue
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "micromouse"))
sys.path.insert(0, str(Path(__file__).parent.parent / "arm"))

import remote_protocol as proto  # noqa: E402
from remote_controller import RemoteController  # noqa: E402
from mobile_base import MobileBase  # noqa: E402
from ball_pickup import ARM_HOME_DEG, ARM_MOVE_TIME_MS, FAN_OFF_PERCENT, RELOAD_HOME_DEG  # noqa: E402

WATCHDOG_TIMEOUT_S = 1.0   # この間メッセージ(ハートビート含む)が来なければ緊急停止
RECV_TIMEOUT_S = 0.2
WORKER_POLL_S = 0.02
RUMBLE_DURATION_MS = 100   # 1コマンド完了ごとにPC側コントローラを振動させる長さ

OPERATION_GUIDE = """\
=== 操作方法 ===
  十字キー上   : 押している間、1区間前進を繰り返す(離すと区間完了後に停止)
  十字キー右/左: 押した瞬間に右/左90度旋回(1回のみ)
  十字キー下   : 押した瞬間に180度旋回(1回のみ)
  △          : 押している間、低速前進
  ○          : 押している間、低速右旋回
  ×          : 押している間、低速後退
  ▢          : 押している間、低速左旋回
  L1          : 押した瞬間にボール回収シーケンスを実行(数秒かかる)
  R1          : 押した瞬間にリロードサーボを180度へ
================
"""


def make_arm_servo():
    """Futaba アームサーボ(mob とは別の Pi UART 接続)を初期化する。

    未接続・シリアルエラー等で失敗しても None を返すだけで続行する
    (L1のボール回収シーケンスだけが使えなくなり、他の操作には影響しない)。
    """
    try:
        from futaba_servo import FutabaServo

        servo = FutabaServo()
        servo.set_torque(True)
        return servo
    except Exception as e:
        print(f"# アームサーボに接続できません(L1は無効になります): {e}")
        return None


def get_local_ip() -> str:
    """LAN 上でこの機体が実際に使っているIPを取得する(UDP接続トリック)。

    software/beacon/discord_ip.py と同じ手法(実際にはパケットを送らない)。
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]


def register_zeroconf(control_port: int, service_name: str):
    from zeroconf import ServiceInfo, Zeroconf

    ip = get_local_ip()
    info = ServiceInfo(
        proto.SERVICE_TYPE,
        f"{service_name}.{proto.SERVICE_TYPE}",
        addresses=[socket.inet_aton(ip)],
        port=control_port,
        properties={},
        server=f"{socket.gethostname()}.local.",
    )
    zc = Zeroconf()
    zc.register_service(info)
    print(f"# zeroconf登録: {info.name}  {ip}:{control_port}")
    return zc, info


def ui_loop(
    should_stop: threading.Event, status: dict, ip_port: str, controller: RemoteController
) -> None:
    """OLED に状態表示し、Lボタンで should_stop を立てる(ベストエフォート)。

    ui_server が無い/接続できない環境では何もせず即座に返る(標準出力での
    運用や自動テストを妨げないため)。
    """
    try:
        from PIL import Image, ImageDraw

        sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))
        from ui_client import UIClient
    except Exception as e:
        print(f"# OLED表示は無効(スキップ): {e}")
        return

    try:
        client = UIClient()
        client.connect(priority=20)
    except Exception as e:
        print(f"# ui_serverに接続できないためOLED表示は無効: {e}")
        return

    try:
        while not should_stop.is_set():
            try:
                buttons = client.get_buttons()
            except Exception:
                break
            if buttons.get("left") in ("pressed", "long_pressed"):
                should_stop.set()
                break

            ball_text = "BALL: OK" if controller.ball_held else "BALL: --"
            lines = ["Manual Control", ip_port, status.get("text", ""), ball_text, "L: Quit"]
            img = Image.new("RGB", (96, 64), "black")
            draw = ImageDraw.Draw(img)
            for i, line in enumerate(lines[:5]):
                draw.text((2, 2 + i * 12), line, fill="white")
            try:
                client.display(img)
            except Exception:
                break
            time.sleep(0.15)
    finally:
        try:
            client.clear()
            client.disconnect()
        except Exception:
            pass


def worker_loop(controller: RemoteController, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        controller.step()
        time.sleep(WORKER_POLL_S)


def _drain_rumble_queue(conn: socket.socket, rumble_queue: "queue.Queue") -> bool:
    """溜まっている振動通知を全て送信する。送信失敗したら False を返す。"""
    while True:
        try:
            duration_ms = rumble_queue.get_nowait()
        except queue.Empty:
            return True
        try:
            conn.sendall(proto.encode_rumble(duration_ms))
        except OSError as e:
            print(f"# 振動通知の送信に失敗: {e}")
            return False


def handle_client(
    conn: socket.socket,
    controller: RemoteController,
    link_lost: threading.Event,
    should_stop: Optional[threading.Event] = None,
    rumble_queue: Optional["queue.Queue"] = None,
) -> None:
    if should_stop is None:
        should_stop = threading.Event()  # 未指定なら「終了要求なし」として扱う
    if rumble_queue is not None:
        # 前回接続時(未接続中)に溜まった古い通知は捨てて開始する
        while True:
            try:
                rumble_queue.get_nowait()
            except queue.Empty:
                break
    conn.settimeout(RECV_TIMEOUT_S)
    buf = b""
    last_msg = time.monotonic()
    try:
        while True:
            if should_stop.is_set():
                print("# 終了要求のため接続を閉じます")
                return
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                if time.monotonic() - last_msg > WATCHDOG_TIMEOUT_S:
                    print("# 通信途絶(watchdog timeout)を検知")
                    return
                if rumble_queue is not None and not _drain_rumble_queue(conn, rumble_queue):
                    return
                continue
            except OSError as e:
                print(f"# ソケットエラー: {e}")
                return

            if not chunk:
                print("# PC側が切断しました")
                return

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = proto.decode_line(line.decode("utf-8", errors="replace"))
                if msg is None:
                    continue
                last_msg = time.monotonic()
                if msg["type"] == proto.MSG_TYPE_BUTTON:
                    controller.on_event(msg["name"], msg["action"])
                # heartbeat はここまでで last_msg 更新済み、他に何もしない

            if rumble_queue is not None and not _drain_rumble_queue(conn, rumble_queue):
                return
    finally:
        link_lost.set()
        controller.handle_disconnect()


def accept_loop(
    server_sock: socket.socket,
    controller: RemoteController,
    link_lost: threading.Event,
    should_stop: threading.Event,
    status: dict,
    rumble_queue: Optional["queue.Queue"] = None,
) -> None:
    server_sock.settimeout(0.5)
    while not should_stop.is_set():
        print("# 接続待機中...")
        status["text"] = "waiting..."
        try:
            conn, addr = server_sock.accept()
        except socket.timeout:
            continue
        print(f"# 接続: {addr}")
        # 改行を含めると draw.text() が1行分の高さのつもりで確保した領域に
        # 2行分描画してしまい、下のBALL/L: Quit行と重なる(OLED表示は
        # 1エントリ=1行という前提のレイアウトのため、必ず1行に収める)。
        status["text"] = f"conn {addr[0]}"
        link_lost.clear()
        try:
            handle_client(conn, controller, link_lost, should_stop, rumble_queue)
        finally:
            conn.close()
        print("# 切断")


def main() -> int:
    ap = argparse.ArgumentParser(description="Daylight 遠隔操作サーバー(機体側)")
    ap.add_argument("--mob-port", default="/dev/ttyUSB0", help="mob シリアルポート")
    ap.add_argument("--mob-baud", type=int, default=3_000_000)
    ap.add_argument("--control-port", type=int, default=proto.DEFAULT_CONTROL_PORT)
    ap.add_argument("--service-name", default="daylight", help="zeroconf 上のサービス名")
    ap.add_argument("--speed-mmps", type=float, default=300.0, help="1区間前進の速度")
    ap.add_argument("--accel-mmps2", type=float, default=1000.0, help="1区間前進の加減速度")
    ap.add_argument("--cell-mm", type=float, default=180.0, help="1区間の距離")
    ap.add_argument("--no-gyro-calibrate", action="store_true", help="起動時のGCALを省略")
    args = ap.parse_args()

    link_lost = threading.Event()
    should_stop = threading.Event()
    base = MobileBase(args.mob_port, args.mob_baud, abort_check=link_lost.is_set)

    # base 構築後は必ず emergency_stop()+close() する(zeroconf未導入等で
    # 途中失敗しても、モータ・シリアルポートを開けっぱなしにしないため)。
    worker_stop = threading.Event()
    worker: Optional[threading.Thread] = None
    ui_thread: Optional[threading.Thread] = None
    zc = None
    info = None
    server_sock: Optional[socket.socket] = None
    arm = None

    try:
        if not args.no_gyro_calibrate:
            print("# ジャイロキャリブレーション中(機体を静止させてください)...")
            try:
                base.gyro_calibrate()
                print("# キャリブレーション完了")
            except Exception as e:
                print(f"# キャリブレーション失敗(続行します): {e}")

        arm = make_arm_servo()
        if arm is not None:
            arm.set_angle(ARM_HOME_DEG, move_time_ms=ARM_MOVE_TIME_MS)

        # 壁センサLEDを有効化(WALL,1)。これが無いと壁センサの差分値が
        # 出ず、1区間前進(stop_at→mobのSTOP)の壁センサFB(lateral
        # correction)が常に無効(error_units=0)のままになる
        # (micromouseのhw_test.py/pattern_test.py/state_machine.pyと同じ
        # 起動時有効化・終了時無効化のパターン)。
        base.wall_led(True)

        rumble_queue: "queue.Queue" = queue.Queue()

        controller = RemoteController(
            base,
            arm=arm,
            cell_speed_mmps=args.speed_mmps,
            cell_accel_mmps2=args.accel_mmps2,
            cell_size_mm=args.cell_mm,
            on_command_done=lambda: rumble_queue.put(RUMBLE_DURATION_MS),
        )

        worker = threading.Thread(
            target=worker_loop, args=(controller, worker_stop), daemon=True
        )
        worker.start()

        zc, info = register_zeroconf(args.control_port, args.service_name)

        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("0.0.0.0", args.control_port))
        server_sock.listen(1)

        print(OPERATION_GUIDE)

        status = {"text": "waiting..."}
        ip_port = f"{get_local_ip()}:{args.control_port}"
        ui_thread = threading.Thread(
            target=ui_loop, args=(should_stop, status, ip_port, controller), daemon=True
        )
        ui_thread.start()

        accept_loop(server_sock, controller, link_lost, should_stop, status, rumble_queue)
    except KeyboardInterrupt:
        print("\n# 終了します")
    except Exception as e:
        print(f"# 起動に失敗しました: {e}")
        return 1
    finally:
        should_stop.set()  # Lボタン以外の終了経路(Ctrl+C・起動失敗等)でもスレッドを止める
        worker_stop.set()
        if worker is not None:
            worker.join(timeout=1.0)
        if ui_thread is not None:
            ui_thread.join(timeout=1.0)
        if zc is not None:
            zc.unregister_service(info)
            zc.close()
        base.emergency_stop()
        # ボール回収機構を安全な既定状態(ファンOFF・リロード/アーム0度)に
        # 戻してから閉じる。close()後は書き込めないため、必ずclose()より前に行う。
        try:
            base.wall_led(False)
            base.set_fan_percent(FAN_OFF_PERCENT)
            base.set_reload_servo(RELOAD_HOME_DEG)
            if arm is not None:
                arm.set_angle(ARM_HOME_DEG, move_time_ms=ARM_MOVE_TIME_MS)
                time.sleep(ARM_MOVE_TIME_MS / 1000.0)
        except Exception as e:
            print(f"# 終了時のリセットに失敗: {e}")
        base.close()
        if arm is not None:
            arm.close()  # トルクオフしてシリアルを閉じる(futaba_servo.pyが自動実施)
        if server_sock is not None:
            server_sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
