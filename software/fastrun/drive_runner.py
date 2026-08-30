"""drive_runner.py — 区間列(Straight/Slalom)を実機で走らせ #T/#COLLIDE を監視する(2026-08-30)。

verify_cells.run_once の走行・監視・要約ロジックを、ミッション走行(mission.py)と
検証(verify_cells)で共有できるよう関数 `drive_segments()` に括り出したもの。挙動は
verify_cells と同一(送信 PCLEAR/PADD/PRUN → #T/#COLLIDE 二重監視 → CSV ログ →
発振要約)。verify 専用の「走行後 180°反転」は含めない(呼び出し側の責務)。

安全(CLAUDE.md「開発の進め方」): 動き出す直前にブザーで予告する(warn_before_move)。
毎走行、位置・角度誤差の発振がないか要約で点検する。
"""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent

from notify import warn_before_move  # noqa: E402
from pattern import Slalom, Straight, send_pattern  # noqa: E402
from verify_loop import _T_RE  # noqa: E402

LOG_DIR = _HERE / "logs"

# 衝突検出(mob の path_collide_* と対応、CLAUDE.md「衝突検出」)。ESP32 側が 1kHz で
# 検出しモーター停止 + #COLLIDE 通知するのが主。ここは二次安全網(#T 20Hz・連続超過)。
COLLIDE_DIST_MM = 150.0
COLLIDE_ANG_RAD = 0.7
COLLIDE_CONSECUTIVE = 3


@dataclass
class DriveResult:
    reached: bool
    collision: Optional[Tuple[int, float, float]]  # (seg, dist_mm, hdg_err_rad)
    log_path: str
    n_samples: int
    max_dist_mm: float
    hdg_sign_changes: int          # hdg_err の符号反転回数(発振の目安)
    end_rtheta_deg: Optional[float]

    @property
    def oscillating(self) -> bool:
        """発振が疑わしいか(符号反転が多い)。呼び出し側の点検補助。"""
        return self.hdg_sign_changes >= 6


def drive_segments(
    link,
    segs: Sequence,
    *,
    tag: str = "mission",
    log_dir: Path = LOG_DIR,
    collide_dist_mm: float = COLLIDE_DIST_MM,
    collide_ang_rad: float = COLLIDE_ANG_RAD,
    collide_consecutive: int = COLLIDE_CONSECUTIVE,
    timeout_s: float = 40.0,
) -> DriveResult:
    """区間列を走らせ、完走 or 衝突まで #T/#COLLIDE を監視して結果を返す。

    前提: 呼び出し側で GCAL・WALL,0・スタートの壁上面補正(向き確定)を済ませてあること。
    この関数は動き出す直前に warn_before_move を鳴らし、send_pattern で走行を開始する。
    """
    log = Path(log_dir) / f"drive_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log.parent.mkdir(parents=True, exist_ok=True)
    n_seg = len(segs)

    warn_before_move(1.0)
    send_pattern(link, segs)

    reached = False
    collision: Optional[Tuple[int, float, float]] = None
    done_hold = 0
    collide_count = 0
    with log.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "type", "seg", "tx", "ty", "rx", "ry",
                    "rtheta_deg", "dist_mm", "hdg_err_rad"])
        t0 = time.monotonic()
        while True:
            raw = link.ser.readline()
            if not raw:
                if time.monotonic() - t0 > timeout_s:
                    print(f"[WARN] {timeout_s:.0f}s timeout、未完了で終了")
                    break
                continue
            line = raw.decode("ascii", errors="replace").strip()
            t = time.monotonic() - t0
            if line.startswith("#COLLIDE"):
                parts = line.split(",")
                c_seg = int(parts[1]) if len(parts) > 1 else -1
                c_dist = float(parts[2]) if len(parts) > 2 else 0.0
                c_hdg = float(parts[3]) if len(parts) > 3 else 0.0
                collision = (c_seg, c_dist, c_hdg)
                w.writerow([f"{t:.3f}", line[:40], "", "", "", "", "", "", "", ""])
                print(f"[COLLIDE] seg={c_seg} dist={c_dist:.1f}mm "
                      f"hdg_err={c_hdg:.3f}rad → 壁衝突として走行中止")
                break
            m = _T_RE.match(line)
            if m:
                seg = int(m.group(1))
                dist = float(m.group(7))
                hdg = float(m.group(8))
                w.writerow([f"{t:.3f}", "#T", seg,
                            m.group(2), m.group(3), m.group(4), m.group(5),
                            f"{float(m.group(6)) * 180 / math.pi:.2f}",
                            m.group(7), m.group(8)])
                if seg >= n_seg:
                    done_hold += 1
                    if done_hold >= 3:
                        reached = True
                        print(f"[DONE] パターン完了(seg>= {n_seg}) @t={t:.2f}s")
                        break
                else:
                    done_hold = 0
                if dist > collide_dist_mm or abs(hdg) > collide_ang_rad:
                    collide_count += 1
                    if collide_count >= collide_consecutive:
                        collision = (seg, dist, hdg)
                        print(f"[COLLIDE#T] seg={seg} dist={dist:.1f}mm "
                              f"hdg_err={hdg:.3f}rad → 壁衝突として走行中止")
                        break
                else:
                    collide_count = 0
                continue
            w.writerow([f"{t:.3f}", line[:40], "", "", "", "", "", "", "", ""])

    link.stop()
    time.sleep(0.2)

    stats = _summarize(log, segs, reached)
    if collision is not None:
        print(f"[衝突] seg={collision[0]} で中止: "
              f"dist={collision[1]:.1f}mm hdg_err={collision[2]:.3f}rad")
    return DriveResult(
        reached=reached,
        collision=collision,
        log_path=str(log),
        n_samples=stats["n"],
        max_dist_mm=stats["max_dist"],
        hdg_sign_changes=stats["sign_changes"],
        end_rtheta_deg=stats["end_rtheta"],
    )


def _summarize(log: Path, segs: Sequence, reached: bool) -> dict:
    """CSV ログから発振・終端の要約を出す(毎走行の点検、CLAUDE.md 安全ルール)。"""
    seg_max: dict[str, float] = {}
    n = 0
    max_dist = 0.0
    sign_changes = 0
    prev_sign = 0
    end_rtheta = None
    with log.open() as f:
        for row in csv.DictReader(f):
            if row.get("type") != "#T":
                continue
            n += 1
            hdg = abs(float(row["hdg_err_rad"]))
            seg_max[row["seg"]] = max(seg_max.get(row["seg"], 0.0), hdg)
            max_dist = max(max_dist, abs(float(row["dist_mm"])))
            v = float(row["hdg_err_rad"])
            s = 1 if v > 0.02 else (-1 if v < -0.02 else 0)
            if s != 0 and prev_sign != 0 and s != prev_sign:
                sign_changes += 1
            if s != 0:
                prev_sign = s
            end_rtheta = row["rtheta_deg"]
    print(f"[まとめ] サンプル{n}件, max|dist|={max_dist:.1f}mm, "
          f"hdg_err符号反転(発振?)={sign_changes}回, reached={reached}")
    for seg in sorted(seg_max, key=int):
        if int(seg) >= len(segs):
            continue
        geo = segs[int(seg)]
        if isinstance(geo, Straight):
            desc = f"直進{geo.distance_mm:.0f}"
        elif isinstance(geo, Slalom):
            desc = f"SL{geo.angle_deg:.0f}{geo.dir}"
        else:
            desc = "?"
        print(f"  seg{seg} [{desc}]: max|hdg_err|="
              f"{math.degrees(seg_max[seg]):.2f}deg")
    return {"n": n, "max_dist": max_dist, "sign_changes": sign_changes,
            "end_rtheta": float(end_rtheta) if end_rtheta is not None else None}
