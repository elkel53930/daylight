#!/usr/bin/env python3
"""ボールセンサ(IO14)のADC生値を0.5秒毎に表示する診断スクリプト。

mob の SEN コマンドを定期的に発行し、レスポンス末尾の ball_raw/ball_det を表示する。

使い方:
    software/venv/bin/python3 software/util/ball_check.py
    software/venv/bin/python3 software/util/ball_check.py --interval 0.2
"""

from __future__ import annotations

import argparse
import time

import serial


def parse_sen_line(line: str) -> tuple[int, int] | None:
    """SEN 行から (ball_raw, ball_det) を取り出す。形式不正なら None。"""
    parts = line.split(",")
    if len(parts) != 13 or parts[0] != "SEN":
        return None
    try:
        return int(parts[11]), int(parts[12])
    except ValueError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="mob のシリアルポート")
    parser.add_argument("--baud", type=int, default=3000000)
    parser.add_argument("--interval", type=float, default=0.5, help="表示間隔[秒](既定0.5秒)")
    args = parser.parse_args()

    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    try:
        while True:
            ser.write(b"SEN\n")
            deadline = time.monotonic() + 1.0
            result = None
            while time.monotonic() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if line.startswith("SEN,"):
                    result = parse_sen_line(line)
                    break

            if result is None:
                print("# SEN応答なし")
            else:
                ball_raw, ball_det = result
                mark = "検出" if ball_det else "  -"
                print(f"ball_raw={ball_raw:4d}  {mark}")

            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
