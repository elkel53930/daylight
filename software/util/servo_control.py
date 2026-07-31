#!/usr/bin/env python3
"""RCサーボ(IO1、MG90S)を角度指定またはトルクオフで操作するスクリプト。

mob (ESP32-S3) の SRV,<angle> / SRVOFF コマンドをシリアルで送信する。

使い方:
    software/venv/bin/python3 software/util/servo_control.py --angle 90
    software/venv/bin/python3 software/util/servo_control.py --off
"""

from __future__ import annotations

import argparse
import time

import serial


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="mob のシリアルポート")
    parser.add_argument("--baud", type=int, default=3000000)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--angle", type=int, help="目標角度[度](0-180)")
    group.add_argument("--off", action="store_true", help="トルクオフ(脱力)")
    args = parser.parse_args()

    if args.angle is not None and not (0 <= args.angle <= 180):
        parser.error("--angle は0-180の範囲で指定してください")

    command = "SRVOFF" if args.off else f"SRV,{args.angle}"

    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    try:
        ser.write((command + "\n").encode("ascii"))
        ser.flush()

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("#SRV"):
                print(line)
                return
        print("# 応答なし(タイムアウト)")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
