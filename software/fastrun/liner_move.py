"""liner_move.py — Liner の既知迷路・点対点の高速移動(L2、2026-08-04)。

Liner は既知迷路+現在姿勢+ゴールセルから経路を計画し、mob の PATTERN 走行
(直進+90°スラローム)で高速移動する。探索はしない(Eiffelが迷路を供給)。

standstill から発進するため、経路の初手が方向転換なら**その場超信地旋回**で
向きを合わせてから直進/スラロームを実行する(スラロームは進入速度が要るので
初手には使わない)。到達は path_controller の #T テレメトリ(seg_index)で検出。
到達後 pose を (goal, 最終向き) に更新する(実位置はドリフトするので、この後
liner_center.recenter_cell で絶対基準へ戻す前提)。

⚠️ 開発ルール: 走行のたびに #T の heading_error 等で発振がないか確認する。
速度は moderate から実機で確認しつつ上げる(PlannerConfig.straight_cruise_mmps)。
"""
from __future__ import annotations

import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# pattern.py は software/mob/ にある(planner.py と同じ経路解決)。planner を
# import する前でも pattern を使えるよう、先に mob をパスへ入れる。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mob"))

from geometry import Direction
from liner_pose import LinerPose, direction_to_gyro_deg
from maze import WallMap
from pattern import send_pattern  # noqa: E402
from planner import PlannerConfig, find_path, plan
import recenter

_T_RE = re.compile(
    r"^#T,(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)"
)


@dataclass
class MoveResult:
    pose: LinerPose            # 到達後の推定姿勢(セル+向き)
    reached: bool              # パターン完了を検出したか
    max_abs_hdg_err_rad: float  # 走行中のヨー誤差の最大絶対値(発振の目安)
    hdg_sign_changes: int      # ヨー誤差の符号反転回数(多いと発振の疑い)
    samples: int


def go_to(link, maze: WallMap, pose: LinerPose, goal: tuple,
          *, cfg: Optional[PlannerConfig] = None,
          settle_s: float = 0.4, timeout_s: float = 25.0) -> MoveResult:
    """pose から goal セルへ既知迷路で高速移動する。到達後の推定姿勢を返す。

    初手が方向転換なら超信地旋回で向きを合わせてから PATTERN を実行する。
    完了は #T の seg_index が区間数に達したら検出。走行中の heading_error で
    発振を監視する(開発ルール)。停止は MOT,0,0。
    """
    cfg = cfg or PlannerConfig()
    if tuple(pose.cell) == tuple(goal):
        return MoveResult(pose=pose, reached=True, max_abs_hdg_err_rad=0.0,
                          hdg_sign_changes=0, samples=0)

    path = find_path(maze, pose.cell, pose.heading, goal, cfg)
    final_dir: Direction = path[-1][2]
    first_dir: Direction = path[1][2]

    # 初手の向きへ超信地旋回(standstill発進。スラロームは進入速度が要るので使わない)。
    if first_dir != pose.heading:
        recenter.turn_to(link, direction_to_gyro_deg(first_dir))
        time.sleep(0.2)

    # turn_to は place_controller の TURN 保持を継続する。次に PRUN(PATH_FOLLOW)へ
    # 移る前に必ず MOT,0,0 で保持を抜ける(旋回が無くても停止済みで無害)。
    link.stop()
    time.sleep(0.15)

    # first_dir を起点に再計画すると初手が直進になり、余分な初手スラロームが出ない。
    segs = plan(maze, pose.cell, first_dir, goal, cfg)
    if not segs:
        return MoveResult(pose=LinerPose(goal[0], goal[1], final_dir), reached=True,
                          max_abs_hdg_err_rad=0.0, hdg_sign_changes=0, samples=0)

    # ⚠️ reset_odometry(RANG)は呼ばない。path_controller.start() が
    # start_heading_rad_=get_angle() を自分で捕捉し以降は相対角度で走るため不要
    # (mob/path_controller.cpp:31,185)。ここで RANG すると odo_ang の絶対ヨー基準
    # (カメラreanchorで確定したもの)が到達時の物理向きで上書きされ、後続の
    # recenter_cell の turn_to(絶対odo依存)が狂う(2026-08-04判明)。距離も自己参照
    # なので RDST も不要。odo_ang は移動後も絶対のまま(ドリフト分のみ)保たれる。
    send_pattern(link, segs)

    # #T を監視: seg_index が区間数に達したら完了。heading_error の発振も監視。
    n_seg = len(segs)
    max_abs = 0.0
    sign_changes = 0
    prev_sign = 0
    samples = 0
    reached = False
    done_hold = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        raw = link.ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        m = _T_RE.match(line)
        if not m:
            continue
        seg = int(m.group(1))
        hdg_err = float(m.group(8))
        samples += 1
        a = abs(hdg_err)
        if a > max_abs:
            max_abs = a
        s = 1 if hdg_err > 0.02 else (-1 if hdg_err < -0.02 else 0)
        if s != 0 and prev_sign != 0 and s != prev_sign:
            sign_changes += 1
        if s != 0:
            prev_sign = s
        if seg >= n_seg:
            done_hold += 1
            if done_hold >= 3:  # 数サンプル保持で確定
                reached = True
                break
        else:
            done_hold = 0

    time.sleep(settle_s)
    link.stop()
    return MoveResult(
        pose=LinerPose(goal[0], goal[1], final_dir),
        reached=reached,
        max_abs_hdg_err_rad=max_abs,
        hdg_sign_changes=sign_changes,
        samples=samples,
    )
