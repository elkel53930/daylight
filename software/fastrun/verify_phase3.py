"""verify_phase3.py — Phase 3 統合チェーン検証: go_to(高速移動) → recenter_cell(L1壁上面補正)。

機体を (0,0) マス中心・北向きに手で置いてから実行する。go_to でゴールセルへ
高速移動し、到達後に壁上面補正でマス中心・最終向きを確定する。補正前後の
offset をログして5mm以内に収まるかを確認する(開発ルール)。

使い方:
    verify_phase3.py --goal 1,1            # 既定: (0,0) 北向き → (1,1)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from camera_align import OnboardCamera  # noqa: E402
from dev_maze import build_dev_maze  # noqa: E402
from geometry import Direction  # noqa: E402
from liner_center import recenter_cell  # noqa: E402
from liner_move import go_to  # noqa: E402
from liner_pose import LinerPose, direction_to_gyro_deg  # noqa: E402
from mob_link import MobLink  # noqa: E402
from notify import warn_before_move  # noqa: E402
import recenter as recenter_ops  # noqa: E402


def recenter_start(link, cam) -> None:
    """(0,0) マス中心・北向きを確定(verify_kanayama.recenter_pose と同じ流れ)。"""
    maze = build_dev_maze()
    pose = LinerPose(0, 0, Direction.N)
    link.stop()
    time.sleep(0.15)
    deg = direction_to_gyro_deg(Direction.N)
    link.send(f"SANG,{math.radians(deg):.5f}")
    if link.wait_for("DONE", timeout_s=1.0) is None:
        raise RuntimeError("recenter: 初回 SANG timeout")
    result = recenter_cell(link, cam, maze, pose)
    link.stop()
    time.sleep(0.15)
    if not (result.get("y") is not None and result["y"].ok
            and result.get("x") is not None and result["x"].ok):
        raise RuntimeError(f"recenter 失敗: {result}")
    recenter_ops.turn_to(link, deg)
    link.stop()
    time.sleep(0.15)
    link.send(f"SANG,{math.radians(deg):.5f}")
    if link.wait_for("DONE", timeout_s=1.0) is None:
        raise RuntimeError("recenter: 北向き SANG timeout")
    print("recenter 完了: (0,0) マス中心・北向き確定")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--goal", default="1,1", help='ゴールセル "x,y"')
    args = ap.parse_args()

    gx, gy = map(int, args.goal.split(","))
    maze = build_dev_maze()
    print(f"go_to (0,0)北向き -> ({gx},{gy})")

    with MobLink(args.port) as link, OnboardCamera() as cam:
        warn_before_move(1.0)
        print("[1/3] GCAL ...")
        if not link.gyro_calibrate():
            raise RuntimeError("GCAL timeout")

        print("[2/3] (0,0) recenter ...")
        recenter_start(link, cam)

        print("[3/3] go_to + recenter_cell ...")
        pose = LinerPose(0, 0, Direction.N)
        from liner_move import go_to_and_recenter
        r = go_to_and_recenter(link, cam, maze, pose, (gx, gy))
        move = r["move"]
        print(f"  move: reached={move.reached} "
              f"max|hdg_err|={math.degrees(move.max_abs_hdg_err_rad):.2f}deg "
              f"sign_changes={move.hdg_sign_changes} pose={move.pose}")
        for axis in ("y", "x"):
            ax = r["recenter"].get(axis)
            if ax is None:
                print(f"  recenter[{axis}]: 壁無し(スキップ)")
            else:
                print(f"  recenter[{axis}]: face={ax.face.name} "
                      f"dist={ax.camera_dist_mm:.1f}mm offset={ax.offset_mm:+.1f}mm ok={ax.ok}")

    print("統合チェーン 完了")


if __name__ == "__main__":
    main()
