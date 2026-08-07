"""verify_diag45.py — Phase 4b 検証: 45°スラローム+斜め直進の実機確認。

開発迷路に唯一存在する安全な斜めコリドー(ユーザー指摘 2026-08-07):
  (0,1)/(0,2) 境界線(y=360) から (1,3)/(2,3) 境界線(x=360) へ。
経路を (0,0) セル中心・北向き起点に書き直した標準パターン:
  直進 232.8mm(北) → 45°右スラローム(R=90) → 斜め直進 344.5mm(NE)
終点 = (360,630)mm = (1,3)/(2,3) 境界線の中点。diag_sim.py で全壁に対する
中心線クリアランス最小 63.6mm を確認済み(機体半幅45mmで余裕18mm)。

走行前: 機体を (0,0) マス中心・北向きに手で置く。走行前にブザー予告する。
走行後: 機体は (360,630)・北東向きで止まる(セル中心ではない)。手で回収し
次の走行はまた (0,0) に置き直す。

使い方:
    verify_diag45.py --dry-run            # 幾何・区間だけ表示(動かさない)
    verify_diag45.py                      # 既定(直進250 / スラローム220 mm/s)
    verify_diag45.py --cruise 300 --slalom 260   # 速度を上げて再検証
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "mob"))

from dev_maze import build_dev_maze  # noqa: E402
from geometry import Direction  # noqa: E402
from liner_center import recenter_cell  # noqa: E402
from liner_pose import LinerPose, direction_to_gyro_deg  # noqa: E402
from mob_link import MobLink  # noqa: E402
from notify import warn_before_move  # noqa: E402
from pattern import Slalom, Straight, send_pattern  # noqa: E402
import recenter as recenter_ops  # noqa: E402
from camera_align import OnboardCamera  # noqa: E402

LOG_DIR = _HERE / "logs"

# 幾何(シミュレーションで検証済み)
STRAIGHT0_MM = 232.8   # (0,0)中心→スラローム接線点 (90,322.8) までの北直進
DIAG_MM = 344.5        # 斜め直進 (116.4,386.4)→(360,630) の長さ
RADIUS_MM = 90.0
ANGLE_DEG = 45.0

_T_RE = re.compile(
    r"^#T,(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)"
)
_SEN_RE = re.compile(
    r"^SEN,([-\d.]+),([-\d.]+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),([-\d.]+),([-\d.]+),(\d+),(\d+)"
)


def recenter_start(link, cam) -> None:
    """(0,0) マス中心・北向きを壁上面補正で確定する。"""
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


def build_segments(cruise: float, slalom: float) -> list:
    """直進→45°右スラローム→斜め直進 の区間列を作る。"""
    return [
        Straight(distance_mm=STRAIGHT0_MM, v_start_mmps=0.0,
                 v_cruise_mmps=cruise, v_end_mmps=slalom),
        Slalom(v_mmps=slalom, dir="R", radius_mm=RADIUS_MM, angle_deg=ANGLE_DEG),
        Straight(distance_mm=DIAG_MM, v_start_mmps=slalom,
                 v_cruise_mmps=cruise, v_end_mmps=0.0),
    ]


def run_once(link, cruise: float, slalom: float, tag: str) -> str:
    log = LOG_DIR / f"verify_diag45_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log.parent.mkdir(parents=True, exist_ok=True)

    warn_before_move(1.0)
    print(f"[1/4] GCAL ...")
    if not link.gyro_calibrate():
        raise RuntimeError("GCAL timeout")

    link.send("WALL,0")  # 壁追従は使わない(純粋な45°スラロームの検証のため)
    time.sleep(0.1)

    print("[2/4] (0,0) recenter ...")
    with OnboardCamera() as cam:
        recenter_start(link, cam)

    segs = build_segments(cruise, slalom)
    print(f"[3/4] パターン {len(segs)} 区間:")
    for s in segs:
        print(f"  {s}")
    print(f"      直進{cruise:.0f}mm/s / スラローム{slalom:.0f}mm/s / "
          f"終点(360,630)mm=((1,3)/(2,3)境界)")

    send_pattern(link, segs)

    n_seg = len(segs)
    rows: list[list] = []
    with log.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "type", "seg", "tx", "ty", "rx", "ry",
                    "rtheta_deg", "dist_mm", "hdg_err_rad",
                    "gyro_z", "vbatt", "ls", "rs", "odo_dist", "odo_ang"])
        t0 = time.monotonic()
        reached = False
        done_hold = 0
        sen_buf: Optional[list] = None
        while True:
            raw = link.ser.readline()
            if not raw:
                if time.monotonic() - t0 > 30.0:
                    print("[WARN] 30s timeout、未完了で終了")
                    break
                continue
            line = raw.decode("ascii", errors="replace").strip()
            t = time.monotonic() - t0
            m = _T_RE.match(line)
            if m:
                seg = int(m.group(1))
                row = [f"{t:.3f}", "#T", seg,
                       m.group(2), m.group(3), m.group(4), m.group(5),
                       f"{float(m.group(6)) * 180 / math.pi:.2f}",
                       m.group(7), m.group(8)]
                if sen_buf:
                    row += sen_buf
                else:
                    row += ["", "", "", "", "", ""]
                w.writerow(row)
                if seg >= n_seg:
                    done_hold += 1
                    if done_hold >= 3:
                        reached = True
                        print(f"[4/4] パターン完了(seg_index>= {n_seg}) @t={t:.2f}s")
                        break
                else:
                    done_hold = 0
                continue
            ms = _SEN_RE.match(line)
            if ms:
                sen_buf = [ms.group(1), ms.group(2), ms.group(5), ms.group(6),
                           ms.group(9), ms.group(10)]
                continue
            w.writerow([f"{t:.3f}", line[:40], "", "", "", "", "", "", "", "",
                        "", "", "", "", "", ""])

    link.stop()
    time.sleep(0.2)

    # 要約
    max_hdg = 0.0
    max_dist = 0.0
    max_ry = 0.0
    n = 0
    end_rx = end_ry = end_rtheta = None
    with log.open() as f:
        for row in csv.DictReader(f):
            if row.get("type") != "#T":
                continue
            n += 1
            max_hdg = max(max_hdg, abs(float(row["hdg_err_rad"])))
            max_dist = max(max_dist, abs(float(row["dist_mm"])))
            max_ry = max(max_ry, abs(float(row["ry"])))
            end_rx, end_ry, end_rtheta = row["rx"], row["ry"], row["rtheta_deg"]
    print(f"[まとめ] サンプル{n}件, max|hdg_err|={math.degrees(max_hdg):.2f}deg, "
          f"max|dist|={max_dist:.1f}mm, max|ry|={max_ry:.1f}mm, reached={reached}")
    if end_rx is not None:
        print(f"        終端(相対) rx={end_rx} ry={end_ry} rtheta={end_rtheta}deg")
    print(f"ログ: {log}")

    # 開発ルール: 動かしたら俯瞰を投稿
    print("俯瞰を撮影して Discord へ投稿します ...")
    try:
        import overhead
        overhead.capture_and_post("verify_diag45 完了: 45°スラローム+斜め直進")
    except Exception as e:  # noqa: BLE001
        print(f"[WARN] 俯瞰投稿失敗: {e}")

    return str(log)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--cruise", type=float, default=250.0, help="直進速度[mm/s]")
    ap.add_argument("--slalom", type=float, default=220.0, help="スラローム速度[mm/s]")
    ap.add_argument("--tag", default="run", help="ログタグ")
    ap.add_argument("--dry-run", action="store_true",
                    help="動かさず区間と終点幾何だけ表示")
    args = ap.parse_args()

    if args.dry_run:
        segs = build_segments(args.cruise, args.slalom)
        for s in segs:
            print(s)
        print(f"終点: (360,630)mm=(1,3)/(2,3)境界中点。走行前は (0,0) 中心・北向きに "
              f"手で置くこと。")
        return

    with MobLink(args.port) as link:
        run_once(link, cruise=args.cruise, slalom=args.slalom, tag=args.tag)


if __name__ == "__main__":
    main()
