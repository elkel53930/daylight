"""verify_kanayama.py — Phase 2 検証: Kanayama 式横位置復元力(path_ky)の実機比較。

既定テストパターン((0,0)北向き → 直進270mm → 右90°スラローム → 直進450mm、
planner 生成・幾何検証済み)を path_ky を変えて走行し、走行中の
#T テレメトリ(seg_index, target/robot 位置, 向き, dist, heading_error)と
SEN(ジャイロ・電圧・壁センサ・odo)をタイムスタンプ付きでファイルへ記録する。

使い方(比較例):
    verify_kanayama.py --tag blend  --ky 0.0        # 旧方式(ベアリングブレンド)
    verify_kanayama.py --tag kanayama --ky 0.004    # 新方式(既定)

ログは software/fastrun/logs/verify_kanayama_<tag>_<timestamp>.csv に保存される。
走行前にブザー予告(notify.warn_before_move)する。機体を (0,0) セル中心・
北向きに手で置いてから実行すること。
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

# fastrun と mob をパスへ
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "mob"))

from geometry import Direction  # noqa: E402
from mob_link import MobLink  # noqa: E402
from notify import warn_before_move  # noqa: E402
from pattern import send_pattern  # noqa: E402
from planner import PlannerConfig, find_path, plan  # noqa: E402
from liner_pose import LinerPose, direction_to_gyro_deg  # noqa: E402
from dev_maze import build_dev_maze  # noqa: E402
from liner_center import recenter_cell  # noqa: E402
import recenter as recenter_ops  # noqa: E402

LOG_DIR = _HERE / "logs"

# #T,seg,tx,ty,rx,ry,rtheta,dist,hdg_err  (20Hz)
_T_RE = re.compile(
    r"^#T,(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)"
)
# SEN,gyro_z,vbatt,lf,ls,rs,rf,enc_r,enc_l,odo_dist,odo_ang,ball_raw,ball_det
_SEN_RE = re.compile(r"^SEN,([-\d.]+),([-\d.]+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),([-\d.]+),([-\d.]+),(\d+),(\d+)")


def recenter_pose(link: MobLink) -> None:
    """(0,0) セル中心・北向きを壁上面補正で確定する(2026-08-05 実機検証済みの
    motion_test.recenter_before_pattern と同じ流れ)。"""
    maze = build_dev_maze()
    pose = LinerPose(0, 0, Direction.N)
    from camera_align import OnboardCamera

    with OnboardCamera() as cam:
        init_heading_deg = direction_to_gyro_deg(Direction.N)
        link.stop()
        time.sleep(0.15)
        link.send(f"SANG,{math.radians(init_heading_deg):.5f}")
        if link.wait_for("DONE", timeout_s=1.0) is None:
            raise RuntimeError("recenter: 初回 SANG timeout")
        result = recenter_cell(link, cam, maze, pose)
        link.stop()
        time.sleep(0.15)

    y_ok = result.get("y") is not None and result["y"].ok
    x_ok = result.get("x") is not None and result["x"].ok
    if not (y_ok and x_ok):
        raise RuntimeError(f"recenter 失敗: y={y_ok} x={x_ok}")

    # 壁上面補正は最後に X 軸面(E/W)で終わるため、PATTERN 開始前に北へ向き直す。
    north_deg = direction_to_gyro_deg(Direction.N)
    recenter_ops.turn_to(link, north_deg)
    link.stop()
    time.sleep(0.15)
    link.send(f"SANG,{math.radians(north_deg):.5f}")
    if link.wait_for("DONE", timeout_s=1.0) is None:
        raise RuntimeError("recenter: 北向き SANG timeout")
    print("recenter 完了: (0,0) マス中心・北向き確定")


def run_once(link: MobLink, ky: float, tag: str) -> str:
    """path_ky を設定して検証パターンを走行し、ログを返す。"""
    log = LOG_DIR / f"verify_kanayama_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log.parent.mkdir(parents=True, exist_ok=True)

    warn_before_move(1.0)
    print(f"[1/4] 走行前準備: GCAL ...")
    if not link.gyro_calibrate():
        raise RuntimeError("GCAL timeout")

    # Kanayama 効果だけを分離するため壁追従は無効化(壁センサ LED OFF)。
    # path_wall_kp はそのままだが、WALL,0 なら ls/rs≈10 < path_wall_present で
    # バイアスが掛からない。
    link.send("WALL,0")
    time.sleep(0.1)

    # 壁上面補正で (0,0) マス中心・北向きを確定してから走行を始める。
    recenter_pose(link)

    # 姿勢: セル(0,0)中心・北向き。オドメトリは path_controller が自己参照するので
    # RANG/RDST はしない(liner_move.go_to と同じ)。recenter で SANG 済み。
    link.send(f"PSET,path_ky,{ky}")
    resp = link.wait_for("#PSET", timeout_s=1.0)
    print(f"[2/4] PSET path_ky -> {resp or '(no resp)'}")
    link.send(f"PGET,path_ky")
    resp = link.wait_for("PVAL,", timeout_s=1.0)
    print(f"       PGET -> {resp or '(no resp)'}")

    # パターン生成(planner ベース)
    wm = build_dev_maze()
    cfg = PlannerConfig(straight_cruise_mmps=400.0, slalom_mmps=360.0)
    segs = plan(wm, (0, 0), Direction.N, (3, 2), cfg)
    print(f"[3/4] パターン {len(segs)} 区間: {[type(s).__name__ for s in segs]}")

    send_pattern(link, segs)

    # #T / SEN を記録しながら完了を待つ
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
                       f"{float(m.group(6))*180/math.pi:.2f}",
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
            # それ以外(エラー・DONE等)も末尾に記録
            w.writerow([f"{t:.3f}", line[:40], "", "", "", "", "", "", "", "",
                        "", "", "", "", "", ""])

    link.stop()
    time.sleep(0.2)

    # 要約: ログから #T の最大 heading_error・dist・ry の絶対最大値を集計
    max_hdg = 0.0
    max_dist = 0.0
    max_ry = 0.0
    n = 0
    with log.open() as f:
        for row in csv.DictReader(f):
            if row.get("type") != "#T":
                continue
            n += 1
            max_hdg = max(max_hdg, abs(float(row["hdg_err_rad"])))
            max_dist = max(max_dist, abs(float(row["dist_mm"])))
            max_ry = max(max_ry, abs(float(row["ry"])))
    print(f"[まとめ] path_ky={ky}: サンプル{n}件, "
          f"max|hdg_err|={math.degrees(max_hdg):.2f}deg, "
          f"max|dist|={max_dist:.1f}mm, max|ry|={max_ry:.1f}mm, "
          f"reached={reached}")
    return str(log)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--ky", type=float, default=0.004,
                    help="path_ky 値(0=旧方式, >0=Kanayama式)")
    ap.add_argument("--tag", default="run", help="ログファイル名のタグ")
    args = ap.parse_args()

    with MobLink(args.port) as link:
        log = run_once(link, ky=args.ky, tag=args.tag)
    print(f"ログ保存: {log}")


if __name__ == "__main__":
    main()
