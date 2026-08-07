"""verify_loop.py — 走行精度検証の周回ループパターン(2026-08-07〜)。

スラローム・斜め走行・長直進を組み合わせた**閉ループ**パターンで、発着点
(0,0) マス中心が一致するため「閉ループ誤差」として走行精度を測れる。

経路(世界座標、diag_sim.py で全壁クリアランス最小63.6mmを検証済み、実証済みの
45°斜め走行と同じ余裕):
  (0,0)中心・北向き
  → 直進232.8(北) → 45°右スラローム(R=90) → 斜め直進344.5(NE) → (360,630)
  → 45°右スラローム → 直進116.4(東) → 90°右スラローム → 直進46.4(南)
  → 90°右スラローム → 直進360(西) → 90°左スラローム → 直進250(南)
  → (0,0)中心・南向き に復帰(閉ループ)

これで以下を一度に確認できる:
  - 45°スラローム(右)・90°スラローム(右/左)の追従精度
  - 斜め直進の向き・位置保持
  - 長直進(360mm)での横ドリフト
  - 発着一致(閉ループ誤差): 最終 #T の rx/ry(=ローカル原点からのズレ)と
    rtheta(=180°からのズレ=正味回転ドリフト)
  - hdg_err の発振(符号反転)の有無

使い方:
    verify_loop.py --dry-run            # 区間・終点幾何だけ表示(動かさない)
    verify_loop.py                      # 既定(直進250 / スラローム220 mm/s)
    verify_loop.py --cruise 300 --slalom 260 --tag lap2   # 速度を上げて再検証

走行前: 機体を (0,0) マス中心・北向きに手で置く。走行前にブザー予告する。
走行後: 機体は (0,0) マス中心・南向きで止まる(ほぼ発着点)。俯瞰を投稿して
閉ループ誤差を目視確認する。次の走行はまた (0,0) に北向きで置き直す。
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

# 閉ループ区間(diag_sim.py で終点 (90,90)・全壁クリアランス63.6mm 検証済み)
SEGMENTS_GEOM = [
    ("S", 232.8),  # 北直進 → (90,322.8)
    ("SL", "R", 45.0),  # 45°右 → (116.4,386.4)
    ("S", 344.5),  # 斜め直進(NE) → (360,630)
    ("SL", "R", 45.0),  # NE→E
    ("S", 116.4),  # 東直進 → (540,656.4)
    ("SL", "R", 90.0),  # E→S
    ("S", 46.4),  # 南直進 → (630,520)
    ("SL", "R", 90.0),  # S→W
    ("S", 360.0),  # 西直進 → (180,430)
    ("SL", "L", 90.0),  # W→S
    ("S", 250.0),  # 南直進 → (90,90)
]

_T_RE = re.compile(
    r"^#T,(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)"
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
    """閉ループの区間列(Straight/Slalom)を作る。"""
    segs = []
    for i, g in enumerate(SEGMENTS_GEOM):
        if g[0] == "S":
            # 前後にスラロームがある直進は v_start=v_end=スラローム速度
            v_start = 0.0 if i == 0 else slalom
            v_end = 0.0 if i == len(SEGMENTS_GEOM) - 1 else slalom
            segs.append(Straight(distance_mm=g[1], v_start_mmps=v_start,
                                 v_cruise_mmps=cruise, v_end_mmps=v_end))
        else:
            segs.append(Slalom(v_mmps=slalom, dir=g[1], radius_mm=90.0,
                               angle_deg=g[2]))
    return segs


def run_once(link, cruise: float, slalom: float, tag: str) -> str:
    log = LOG_DIR / f"verify_loop_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log.parent.mkdir(parents=True, exist_ok=True)

    warn_before_move(1.0)
    print(f"[1/4] GCAL ...")
    if not link.gyro_calibrate():
        raise RuntimeError("GCAL timeout")

    link.send("WALL,0")  # 壁追従は使わない(純粋なスラローム/斜め走行の検証のため)
    time.sleep(0.1)

    print("[2/4] (0,0) recenter ...")
    with OnboardCamera() as cam:
        recenter_start(link, cam)

    segs = build_segments(cruise, slalom)
    print(f"[3/4] パターン {len(segs)} 区間(閉ループ、発着 (0,0)中心):")
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

    # 要約(セグメント別 + 閉ループ誤差)
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
          f"hdg_err符号反転={sign_changes}回, reached={reached}")
    for seg in sorted(seg_max, key=int):
        idx = int(seg)
        if idx >= len(SEGMENTS_GEOM):
            continue  # 完了後に流れる seg_index>=区間数 は区間でない
        geo = SEGMENTS_GEOM[idx]
        if geo[0] == "S":
            desc = f"直進{geo[1]:.0f}"
        else:
            desc = f"SL{geo[2]:.0f}{geo[1]}"
        print(f"  seg{idx} [{desc}]: max|hdg_err|={math.degrees(seg_max[seg]):.2f}deg")
    if end_rx is not None:
        rx, ry = float(end_rx), float(end_ry)
        rtheta = float(end_rtheta)
        # 理想終端 heading は -180°(南)。-180°からの角度距離を正規化して出す。
        dev = abs(((rtheta + 180.0) + 180.0) % 360.0 - 180.0)
        print(f"[閉ループ誤差] 終端 rx={rx:.1f} ry={ry:.1f} (理想 0,0, "
              f"距離{math.hypot(rx, ry):.1f}mm), rtheta={rtheta:.1f}deg "
              f"(理想 -180, ズレ{dev:.1f}deg)")
    print(f"ログ: {log}")

    # 開発ルール: 動かしたら俯瞰を投稿
    print("俯瞰を撮影して Discord へ投稿します ...")
    try:
        import overhead
        overhead.capture_and_post("verify_loop 完了: 閉ループ(45°+斜め+90°R/L)")
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
        for i, s in enumerate(segs):
            print(f"[{i}] {s}")
        print("閉ループ: 発着 (0,0) マス中心。走行前は (0,0) 中心・北向きに手で置くこと。")
        print("完走後は (0,0) 中心・南向きに復帰(閉ループ誤差で測定)。")
        return

    with MobLink(args.port) as link:
        run_once(link, cruise=args.cruise, slalom=args.slalom, tag=args.tag)


if __name__ == "__main__":
    main()
