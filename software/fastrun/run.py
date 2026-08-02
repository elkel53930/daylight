"""run.py — 走行計画の生成・表示・実行 CLI(2026-08-03〜)。

使い方:
    # 計画を表示するだけ(ハード不要)
    python3 run.py --start 0,0 --dir N --goal 3,3 --maze maze.txt

    # 実機で走行(空マップで直進スモークテスト)
    python3 run.py --start 0,0 --dir N --goal 0,2 --empty 1x4 --run

座標は "x,y"、向きは N/E/S/W。--maze はASCIIアートの壁マップ(maze.from_ascii)。
--empty WxH は壁なしの WxH マップ(直進スモークテスト用)。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from geometry import Direction
from maze import WallMap
from planner import PlannerConfig, plan
from pattern import Slalom, Straight, send_pattern


def _parse_cell(s: str):
    x, y = s.split(",")
    return int(x), int(y)


def _build_maze(args) -> WallMap:
    if args.maze:
        return WallMap.from_ascii(Path(args.maze).read_text())
    if args.empty:
        w, h = args.empty.lower().split("x")
        return WallMap(int(w), int(h))
    raise SystemExit("--maze か --empty のどちらかを指定してください")


def _describe(segs) -> None:
    print(f"# {len(segs)} 区間:")
    for i, s in enumerate(segs):
        if isinstance(s, Straight):
            print(
                f"  [{i}] STRAIGHT {s.distance_mm:6.1f}mm "
                f"v:{s.v_start_mmps:.0f}->{s.v_cruise_mmps:.0f}->{s.v_end_mmps:.0f}"
            )
        elif isinstance(s, Slalom):
            print(
                f"  [{i}] SLALOM   {s.dir} r{s.radius_mm:.0f} {s.angle_deg:.0f}deg "
                f"@{s.v_mmps:.0f}mm/s"
            )


def _execute(segs, port: str, monitor_s: float) -> None:
    from mob_link import MobLink

    with MobLink(port) as link:
        sen = link.read_sen()
        if sen is None:
            print("!! SEN応答なし。ポート・電源を確認して再実行してください。")
            return
        print(f"接続OK vbatt={sen['vbatt']:.2f}V")
        print("GCAL(静止確認)...")
        link.gyro_calibrate()
        link.reset_odometry()
        print("走行開始 (PCLEAR/PADD/PRUN)...")
        send_pattern(link, segs)
        t0 = time.monotonic()
        while time.monotonic() - t0 < monitor_s:
            sen = link.read_sen()
            if sen:
                print(
                    f"  t={time.monotonic()-t0:4.1f}s "
                    f"odo_dist={sen['odo_dist']:7.1f}mm odo_ang={sen['odo_ang']:+.3f}rad"
                )
            time.sleep(0.2)
        link.stop()
        print("停止 (MOT,0,0)")


def main() -> None:
    ap = argparse.ArgumentParser(description="fastrun 走行計画 CLI")
    ap.add_argument("--start", required=True, help='"x,y" 現在セル')
    ap.add_argument("--dir", required=True, choices=[d.name for d in Direction], help="現在の向き")
    ap.add_argument("--goal", required=True, help='"x,y" ゴールセル')
    ap.add_argument("--maze", help="ASCIIアート壁マップのファイル")
    ap.add_argument("--empty", help='壁なしマップ "WxH"(スモークテスト用)')
    ap.add_argument("--straight-mmps", type=float, default=400.0)
    ap.add_argument("--slalom-mmps", type=float, default=360.0)
    ap.add_argument("--run", action="store_true", help="実機で走行する")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--monitor-s", type=float, default=8.0)
    args = ap.parse_args()

    wm = _build_maze(args)
    cfg = PlannerConfig(
        straight_cruise_mmps=args.straight_mmps,
        slalom_mmps=args.slalom_mmps,
    )
    segs = plan(
        wm,
        _parse_cell(args.start),
        Direction[args.dir],
        _parse_cell(args.goal),
        cfg,
    )
    _describe(segs)

    if args.run:
        _execute(segs, args.port, args.monitor_s)


if __name__ == "__main__":
    main()
