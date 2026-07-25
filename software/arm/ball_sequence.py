#!/usr/bin/env python3
"""ball_sequence.py - ボール回収シーケンス

リロードサーボ(ESP32 経由 PWM の RC サーボ、mob の SRV コマンド)、
アームサーボ(Futaba シリアルサーボ、software/arm/futaba_servo.py)、
吸引ファン(mob の FAN コマンド)、ボールセンサ(mob の SEN 応答 ball_raw)を
連携させて以下のシーケンスを実行する:

  1. アームサーボ・リロードサーボを 0 度へ
  2. 0.5 秒待つ
  3. リロードサーボを 140 度へ
  4. アームサーボを 1000ms で 103 度へ
  5. アームサーボが 103 度に到達(動かし始めて 1000ms 経過)したらファン Duty 50%
  6. ボールセンサ値が 100 を超えたらアームサーボを 1000ms で 0 度へ
  7. アームサーボが 0 度に到達(動かし始めて 1000ms 経過)したらファン Duty 0%
  8. 0.2 秒待つ
  9. アームサーボを 200ms で 15 度へ動かし、15 度に到達するまで待つ
 10. リロードサーボを 180 度へ

mob(/dev/ttyUSB0, 3,000,000bps)と Futaba(/dev/ttyAMA0)は別ポート。
SRV/FAN は DONE を返さない送りっぱなしコマンド、SEN は read を先に
始めてから送らないと応答を取りこぼす(CLAUDE.md 参照)。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import serial

sys.path.insert(0, str(Path(__file__).parent))
from futaba_servo import FutabaServo


def duty_percent_to_byte(percent: float) -> int:
    """Duty [%] を FAN コマンドの 0-255 に変換する。"""
    return max(0, min(255, round(255 * percent / 100.0)))


class MobLink:
    """mob(ESP32-S3)へ SRV/FAN を送り、SEN から ball_raw を読む簡易ドライバ。"""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 3_000_000):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def set_reload_servo(self, angle_deg: int) -> None:
        """リロードサーボ(RC サーボ)角度設定。DONE 応答なし。"""
        self.ser.write(f"SRV,{int(angle_deg)}\n".encode("ascii"))
        self.ser.flush()

    def set_fan_percent(self, percent: float) -> None:
        """吸引ファン Duty 設定。DONE 応答なし。"""
        self.ser.write(f"FAN,{duty_percent_to_byte(percent)}\n".encode("ascii"))
        self.ser.flush()

    def read_ball_raw(self, timeout_s: float = 1.0) -> Optional[int]:
        """SEN を要求し ball_raw(12 フィールド目)を返す。取れなければ None。"""
        self.ser.reset_input_buffer()
        self.ser.write(b"SEN\n")
        self.ser.flush()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line.startswith("SEN,"):
                continue
            parts = line.split(",")
            if len(parts) != 13:
                continue
            try:
                return int(parts[11])  # ball_raw
            except ValueError:
                continue
        return None

    def wait_ball_above(
        self, threshold: int, timeout_s: float = 3.0, poll_interval_s: float = 0.02
    ) -> Optional[int]:
        """ball_raw が threshold を超えるまでポーリングし、その値を返す。

        timeout_s を超えても検出できなければ None を返す。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            value = self.read_ball_raw()
            if value is not None and value > threshold:
                return value
            time.sleep(poll_interval_s)
        return None


def run_sequence(mob_port: str, servo_port: str, ball_threshold: int) -> None:
    mob = MobLink(port=mob_port)
    arm = FutabaServo(port=servo_port)
    try:
        arm.set_torque(True)

        # 1. アームサーボ・リロードサーボを 0 度へ
        print("[1] アームサーボ・リロードサーボ → 0 度")
        arm.set_angle(0)
        mob.set_reload_servo(0)

        # 2. 0.5 秒待つ
        time.sleep(0.5)

        # 3. リロードサーボを 140 度へ
        print("[3] リロードサーボ → 140 度")
        mob.set_reload_servo(140)

        # 4. アームサーボを 1000ms で 103 度へ
        print("[4] アームサーボ → 103 度 (1000ms)")
        arm_move_start = time.monotonic()
        arm.set_angle(103, move_time_ms=1000)

        # 5. 動かし始めて 1000ms 経過(103 度到達)したらファン Duty 50%
        remaining = 1.0 - (time.monotonic() - arm_move_start)
        if remaining > 0:
            time.sleep(remaining)
        print("[5] ファン Duty → 50%")
        mob.set_fan_percent(50)

        # 6. ボールセンサ値が 100 を超えたらアームサーボを 1000ms で 0 度へ(最大 3 秒待機)
        print(f"[6] ボールセンサ待機 (ball_raw > {ball_threshold}, 最大3秒) ...")
        detected = mob.wait_ball_above(ball_threshold, timeout_s=3.0)
        if detected is not None:
            print(f"    ボール検出 (ball_raw={detected})")
        else:
            print("    タイムアウト(3秒)。検出なしで次へ進む")
        print("    アームサーボ → 0 度 (1000ms)")
        arm_move_start = time.monotonic()
        arm.set_angle(0, move_time_ms=1000)

        # 7. 動かし始めて 1000ms 経過(0 度到達)したらファン Duty 0%
        remaining = 1.0 - (time.monotonic() - arm_move_start)
        if remaining > 0:
            time.sleep(remaining)
        print("[7] ファン Duty → 0%")
        mob.set_fan_percent(0)

        # 8. 0.2 秒待つ
        time.sleep(0.2)

        # 9. アームサーボを 15 度へ動かし、15 度に到達するまで待つ
        print("[9] アームサーボ → 15 度 (200ms)")
        arm_move_start = time.monotonic()
        arm.set_angle(15, move_time_ms=200)
        remaining = 0.2 - (time.monotonic() - arm_move_start)
        if remaining > 0:
            time.sleep(remaining)

        # 10. リロードサーボを 180 度へ
        print("[10] リロードサーボ → 180 度")
        mob.set_reload_servo(180)

        print("シーケンス完了")
    finally:
        arm.close()  # __del__ でもトルクオフされるが明示的に閉じる
        mob.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="ボール回収シーケンス")
    ap.add_argument("--mob-port", default="/dev/ttyUSB0", help="mob シリアルポート")
    ap.add_argument("--servo-port", default="/dev/ttyAMA0", help="Futaba サーボポート")
    ap.add_argument("--ball-threshold", type=int, default=100,
                    help="ボール検出しきい値(ball_raw の生値)")
    args = ap.parse_args()
    run_sequence(args.mob_port, args.servo_port, args.ball_threshold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
