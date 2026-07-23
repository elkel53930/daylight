#!/usr/bin/env python3
"""左右モーターを正転10秒→停止1秒→逆転10秒→停止1秒で繰り返し駆動し、
回転数[rpm]を1秒毎に表示する診断スクリプト。

mob (ESP32-S3) の DUTY,<r>,<l> コマンド(生duty直指令、速度PID非経由)で
モーターを駆動し、SEN のエンコーダ値(14bit, 0-16383)の差分から回転数を
算出する。ギア滑りなどハードウェアの健全性を、速度PIDの補正に隠されない
形で確認するためのもの。

使い方:
    software/venv/bin/python3 software/util/motor_check.py
    software/venv/bin/python3 software/util/motor_check.py --duty-pct 40 --duration 60
"""

from __future__ import annotations

import argparse
import time

import serial

DUTY_MAX = 1023
ENCODER_RESOLUTION = 16384  # 14bit


def delta_14bit(now: int, prev: int) -> int:
    """14bitエンコーダのラップアラウンドを考慮した符号付き差分。"""
    d = (now - prev) % ENCODER_RESOLUTION
    if d > ENCODER_RESOLUTION // 2:
        d -= ENCODER_RESOLUTION
    return d


def parse_sen_line(line: str) -> tuple[int, int] | None:
    """SEN 行から (enc_r, enc_l) を取り出す。形式不正なら None。"""
    parts = line.split(",")
    if len(parts) != 11 or parts[0] != "SEN":
        return None
    try:
        return int(parts[7]), int(parts[8])
    except ValueError:
        return None


def read_encoders(ser: serial.Serial, timeout_s: float = 2.0) -> tuple[int, int] | None:
    """SEN を要求し、エンコーダ値を1フレーム読む。"""
    ser.write(b"SEN\n")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if line.startswith("SEN,"):
            parsed = parse_sen_line(line)
            if parsed is not None:
                return parsed
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="mob のシリアルポート")
    parser.add_argument("--baud", type=int, default=3000000)
    parser.add_argument("--duty-pct", type=float, default=25.0, help="duty比[%%](既定25%%)")
    parser.add_argument("--run-s", type=float, default=10.0, help="正転/逆転それぞれの継続時間[秒](既定10秒)")
    parser.add_argument("--pause-s", type=float, default=1.0, help="正転/逆転の間の停止時間[秒](既定1秒)")
    parser.add_argument("--duration", type=float, default=None, help="実行時間[秒](未指定なら Ctrl+C まで繰り返す)")
    args = parser.parse_args()

    duty = round(DUTY_MAX * args.duty_pct / 100.0)
    print(f"duty = +-{duty} ({args.duty_pct:.1f}% of +-{DUTY_MAX})")

    # (ラベル, duty, 継続時間[秒])。この順で無限に繰り返す。
    phases = [
        ("FWD", duty, args.run_s),
        ("STOP", 0, args.pause_s),
        ("REV", -duty, args.run_s),
        ("STOP", 0, args.pause_s),
    ]

    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    try:
        prev = read_encoders(ser)
        if prev is None:
            raise RuntimeError("SEN応答を取得できませんでした")
        prev_r, prev_l = prev
        prev_t = time.monotonic()
        start_t = prev_t

        phase_idx = 0
        label, phase_duty, phase_len_s = phases[phase_idx]
        print(f"-- {label} --")
        ser.write(f"DUTY,{phase_duty},{phase_duty}\n".encode("ascii"))
        phase_start_t = prev_t

        while args.duration is None or (time.monotonic() - start_t) < args.duration:
            time.sleep(1.0)
            now_t = time.monotonic()

            # フェーズ切り替え(残り時間の端数は次の1秒待ちに乗せる)
            if now_t - phase_start_t >= phase_len_s:
                phase_idx = (phase_idx + 1) % len(phases)
                label, phase_duty, phase_len_s = phases[phase_idx]
                print(f"-- {label} --")
                ser.write(f"DUTY,{phase_duty},{phase_duty}\n".encode("ascii"))
                phase_start_t = now_t

            now = read_encoders(ser)
            if now is None:
                print("# SEN応答なし、スキップ")
                continue
            now_r, now_l = now
            dt_s = now_t - prev_t

            dr = delta_14bit(now_r, prev_r)
            dl = delta_14bit(now_l, prev_l)
            rpm_r = (dr / ENCODER_RESOLUTION) * (60.0 / dt_s)
            rpm_l = (dl / ENCODER_RESOLUTION) * (60.0 / dt_s)

            print(f"R: {rpm_r:+7.1f} rpm   L: {rpm_l:+7.1f} rpm")

            prev_r, prev_l = now_r, now_l
            prev_t = now_t
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"DUTY,0,0\n")
        ser.close()


if __name__ == "__main__":
    main()
