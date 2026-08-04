#!/usr/bin/env python3
"""壁上面補正を実機で実行する簡易CLI。"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from camera_align import OnboardCamera
from dev_maze import build_dev_maze
from geometry import Direction
from liner_center import recenter_cell
from liner_pose import LinerPose
from mob_link import MobLink


def main() -> None:
    ap = argparse.ArgumentParser(description="Liner の壁上面補正を実行")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="mob のシリアルポート")
    ap.add_argument("--cell", default="0,0", help='現在セル "x,y"')
    ap.add_argument("--dir", default="N", choices=[d.name for d in Direction], help="現在向き")
    ap.add_argument("--maze", choices=["dev"], default="dev", help="迷路定義(dev=開発用4x4)")
    args = ap.parse_args()

    if args.maze == "dev":
        maze = build_dev_maze()
    else:
        raise SystemExit("未対応の maze 指定です")

    x, y = map(int, args.cell.split(","))
    pose = LinerPose(x, y, Direction[args.dir])

    print(f"start recenter: port={args.port} cell=({x},{y}) dir={args.dir}")
    with MobLink(args.port) as link, OnboardCamera() as cam:
        recenter_cell(link, cam, maze, pose)

        # 終了時に ESP32 側の継続制御を確実にオフにする。
        print("send stop command to ESP32")
        link.stop()
        time.sleep(0.15)
    print("recenter done")


if __name__ == "__main__":
    main()
