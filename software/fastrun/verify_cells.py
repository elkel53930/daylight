"""verify_cells.py — マス列(セル列)パターンの実機走行検証(2026-08-08〜)。

pattern_from_cells が生成する区間列(斜めショートカット含む)を実機で走らせ、
#T ログで位置・速度・角度・角速度(発振の有無)を確認する。verify_loop の
実行フロー(GCAL → WALL,0 → 壁上面補正でスタート位置確定 → ブザー予告 →
走行 → #T 監視 → 180°旋回で反転)を再利用する。

使い方:
    verify_cells.py --dry-run                     # 区間だけ表示(動かさない)
    verify_cells.py --cells "0,0,0,1,0,2,1,2,1,3" # 既定速度で実機走行
    verify_cells.py --cruise 300 --slalom 260 --tag lap1

走行前: 機体を (0,0) マス中心・北向きに手で置く(recenter が壁上面で確定する)。
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import List, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "mob"))

from dev_maze import build_dev_maze  # noqa: E402
from geometry import Direction  # noqa: E402
from liner_pose import direction_to_gyro_deg  # noqa: E402
from mob_link import MobLink  # noqa: E402
from notify import warn_before_move  # noqa: E402
from pattern import Slalom, Straight, send_pattern  # noqa: E402
from planner import PlannerConfig, pattern_from_cells  # noqa: E402
from verify_loop import _T_RE, recenter_start  # noqa: E402
import recenter as recenter_ops  # noqa: E402

LOG_DIR = _HERE / "logs"

# 走行後のスタート位置復帰の向き(パターン終端 heading の基準)。
# ここではパターンが北向きで終わるため、180°旋回で南向き→北向きへ戻す。
TARGET_END_HEADING_DEG = 0.0  # 北


def parse_cells(s: str) -> List[Tuple[int, int]]:
    parts = [int(v) for v in s.split(",")]
    if len(parts) < 4 or len(parts) % 2:
        raise SystemExit("--cells は x,y の偶数個の数値列(例: 0,0,0,1,0,2,1,2)")
    return [(parts[i], parts[i + 1]) for i in range(0, len(parts), 2)]


def build_segments(cells: List[Tuple[int, int]], cruise: float,
                   slalom: float) -> List:
    cfg = PlannerConfig(straight_cruise_mmps=cruise, slalom_mmps=slalom)
    return pattern_from_cells(cells, Direction.N, cfg=cfg,
                              wm=build_dev_maze())


def run_once(link, cells: List[Tuple[int, int]], cruise: float, slalom: float,
             tag: str) -> str:
    log = LOG_DIR / f"verify_cells_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log.parent.mkdir(parents=True, exist_ok=True)

    warn_before_move(1.0)
    print("[1/4] GCAL ...")
    if not link.gyro_calibrate():
        raise RuntimeError("GCAL timeout")

    link.send("WALL,0")  # 壁追従は使わない(斜めショートカットの検証のため)
    time.sleep(0.1)

    print("[2/4] (0,0) recenter ...")
    from camera_align import OnboardCamera
    with OnboardCamera() as cam:
        recenter_start(link, cam)

    segs = build_segments(cells, cruise, slalom)
    print(f"[3/4] パターン {len(segs)} 区間 (マス列 {len(cells)} セル):")
    for i, s in enumerate(segs):
        print(f"  [{i}] {s}")
    print(f"      直進{cruise:.0f}mm/s / スラローム{slalom:.0f}mm/s")

    send_pattern(link, segs)

    n_seg = len(segs)
    rows: list[list] = []
    with log.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "type", "seg", "tx", "ty", "rx", "ry",
                    "rtheta_deg", "dist_mm", "hdg_err_rad"])
        t0 = time.monotonic()
        reached = False
        done_hold = 0
        while True:
            raw = link.ser.readline()
            if not raw:
                if time.monotonic() - t0 > 40.0:
                    print("[WARN] 40s timeout、未完了で終了")
                    break
                continue
            line = raw.decode("ascii", errors="replace").strip()
            t = time.monotonic() - t0
            m = _T_RE.match(line)
            if m:
                seg = int(m.group(1))
                w.writerow([f"{t:.3f}", "#T", seg,
                            m.group(2), m.group(3), m.group(4), m.group(5),
                            f"{float(m.group(6)) * 180 / math.pi:.2f}",
                            m.group(7), m.group(8)])
                if seg >= n_seg:
                    done_hold += 1
                    if done_hold >= 3:
                        reached = True
                        print(f"[4/4] パターン完了(seg_index>= {n_seg}) @t={t:.2f}s")
                        break
                else:
                    done_hold = 0
                continue
            w.writerow([f"{t:.3f}", line[:40], "", "", "", "", "", "", "", ""])

    link.stop()
    time.sleep(0.2)

    # 要約(区間別 max|hdg_err| + 発振(符号反転)回数 + 終端)
    seg_max: dict[str, float] = {}
    n = 0
    max_dist = 0.0
    sign_changes = 0
    prev_sign = 0
    end_rx = end_ry = end_rtheta = None
    with log.open() as f:
        for row in csv.DictReader(f):
            if row.get("type") != "#T":
                continue
            n += 1
            hdg = abs(float(row["hdg_err_rad"]))
            seg_max[row["seg"]] = max(seg_max.get(row["seg"], 0.0), hdg)
            max_dist = max(max_dist, abs(float(row["dist_mm"])))
            s = 1 if float(row["hdg_err_rad"]) > 0.02 else (
                -1 if float(row["hdg_err_rad"]) < -0.02 else 0)
            if s != 0 and prev_sign != 0 and s != prev_sign:
                sign_changes += 1
            if s != 0:
                prev_sign = s
            end_rx, end_ry, end_rtheta = row["rx"], row["ry"], row["rtheta_deg"]
    print(f"[まとめ] サンプル{n}件, max|dist|={max_dist:.1f}mm, "
          f"hdg_err符号反転(発振?)={sign_changes}回, reached={reached}")
    for seg in sorted(seg_max, key=int):
        if int(seg) >= len(segs):
            continue
        geo = segs[int(seg)]
        if isinstance(geo, Straight):
            desc = f"直進{geo.distance_mm:.0f}"
        else:
            desc = f"SL{geo.angle_deg:.0f}{geo.dir}"
        print(f"  seg{seg} [{desc}]: max|hdg_err|={math.degrees(seg_max[seg]):.2f}deg")
    if end_rx is not None:
        rx, ry = float(end_rx), float(end_ry)
        rtheta = float(end_rtheta)
        dev = abs(((rtheta + 180.0) + 180.0) % 360.0 - 180.0)
        print(f"[終端] rx={rx:.1f} ry={ry:.1f} (オドメトリ基準), "
              f"rtheta={rtheta:.1f}deg (北=0 から {abs(rtheta):.1f}deg)")
    print(f"ログ: {log}")

    # 走行後の反転: 完走時は北向き(rtheta≈0°)なので、超信地旋回180°で南向きへ
    # 戻す。向きは recenter のケーブルよじれ対策(net_rotation_deg)で左右バランス
    # を選ぶ(turn_to は TURN保持のまま返るため、完了後に stop で抜ける)。
    print("180°旋回で反転します ...")
    recenter_ops.turn_to(link, direction_to_gyro_deg(Direction.S))
    link.stop()
    print("反転完了: 南向き")

    return str(log)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--cells", default="0,0,0,1,0,2,1,2,1,3",
                    help="マス列 \"x,y,...\"(既定: (0,0)(0,1)(0,2)(1,2)(1,3))")
    ap.add_argument("--cruise", type=float, default=400.0, help="直進速度[mm/s]")
    ap.add_argument("--slalom", type=float, default=360.0, help="スラローム速度[mm/s]")
    ap.add_argument("--tag", default="run", help="ログタグ")
    ap.add_argument("--dry-run", action="store_true",
                    help="動かさず区間だけ表示")
    args = ap.parse_args()

    cells = parse_cells(args.cells)
    segs = build_segments(cells, args.cruise, args.slalom)
    print(f"マス列: {cells}")
    for i, s in enumerate(segs):
        print(f"  [{i}] {s}")

    if args.dry_run:
        print("走行前は (0,0) マス中心・北向きに手で置くこと。")
        return

    with MobLink(args.port) as link:
        run_once(link, cells, cruise=args.cruise, slalom=args.slalom,
                 tag=args.tag)


if __name__ == "__main__":
    main()
