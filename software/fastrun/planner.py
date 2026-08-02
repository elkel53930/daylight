"""planner.py — 走行計画エンジン(2026-08-03〜)。

入力:
  - 壁マップ (WallMap)
  - ロボットの現在セル (start_x, start_y) と現在の向き (start_dir)
  - ゴールのセル (goal_x, goal_y)

出力:
  - pattern.py の Straight / Slalom 区間のリスト
    (そのまま PCLEAR/PADD/PRUN で mob へ送って走行できる)

探索は (セル, 向き) を状態としたダイクストラ法。エッジのコストを
「直進の所要時間 + ターンの所要時間(角度に応じた重み)」にすることで、
最短距離ではなく最短時間(ターンが少なく直線的な経路)を選ぶ。将来
斜め走行の状態・エッジを足しても同じ枠組みで拡張できる(Phase 4)。

区間生成では、90°スラロームが旋回半径 R 分だけ前後の直進を「食う」
(接線長 = R for 90°)ことを考慮して直進距離を短縮する。標準的な
クラシックマウスのスラローム区間割り。
"""

from __future__ import annotations

import heapq
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

from geometry import CELL_MM, Direction, slalom_dir_symbol, turn_between
from maze import WallMap

# pattern.py(software/mob/)の Straight/Slalom を再利用する。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mob"))
from pattern import Segment, Slalom, Straight  # noqa: E402


@dataclass(frozen=True)
class PlannerConfig:
    straight_cruise_mmps: float = 400.0   # 直進の巡航速度(ユーザー目標)
    slalom_mmps: float = 360.0            # スラローム旋回時の速度(R90で安全な値、実機で要調整)
    slalom_radius_mm: float = 90.0        # 90°スラローム半径(半セル)
    turn_time_penalty_s: float = 0.35     # 経路探索用: 90°ターン1回の時間ペナルティ
    uturn_time_penalty_s: float = 0.9     # 経路探索用: 180°Uターンの時間ペナルティ


# ----- 経路探索 -----

State = Tuple[int, int, Direction]  # (x, y, 向き)


def _edge_time(cfg: PlannerConfig, turn_steps: int) -> float:
    """1セル進むエッジの所要時間の見積り(探索コスト)。"""
    straight_t = CELL_MM / cfg.straight_cruise_mmps
    if turn_steps == 0:
        return straight_t
    if turn_steps == 2:
        return straight_t + cfg.uturn_time_penalty_s
    return straight_t + cfg.turn_time_penalty_s


def find_path(
    wm: WallMap,
    start: Tuple[int, int],
    start_dir: Direction,
    goal: Tuple[int, int],
    cfg: Optional[PlannerConfig] = None,
) -> List[State]:
    """ダイクストラで (セル, 向き) 状態の最短時間経路を返す。

    戻り値は状態の列 [(sx,sy,start_dir), ..., (gx,gy,final_dir)]。
    先頭はスタート(まだ動いていない)、以降は各セルへ「その向きで進入」
    した状態。経路が無ければ ValueError。
    """
    cfg = cfg or PlannerConfig()
    sx, sy = start
    gx, gy = goal
    start_state: State = (sx, sy, start_dir)

    dist: dict[State, float] = {start_state: 0.0}
    prev: dict[State, Optional[State]] = {start_state: None}
    pq: List[Tuple[float, int, State]] = [(0.0, 0, start_state)]
    counter = 0
    goal_state: Optional[State] = None

    while pq:
        d, _, (x, y, h) = heapq.heappop(pq)
        if d > dist.get((x, y, h), math.inf):
            continue
        if (x, y) == (gx, gy):
            goal_state = (x, y, h)
            break
        for nd in Direction:
            if not wm.can_move(x, y, nd):
                continue
            dx, dy = nd.delta
            nx, ny = x + dx, y + dy
            turn_steps = turn_between(h, nd)
            nd_state: State = (nx, ny, nd)
            nd_cost = d + _edge_time(cfg, turn_steps)
            if nd_cost < dist.get(nd_state, math.inf):
                dist[nd_state] = nd_cost
                prev[nd_state] = (x, y, h)
                counter += 1
                heapq.heappush(pq, (nd_cost, counter, nd_state))

    if goal_state is None:
        raise ValueError(f"経路が見つからない: {start}{start_dir.name} -> {goal}")

    # 経路復元
    path: List[State] = []
    s: Optional[State] = goal_state
    while s is not None:
        path.append(s)
        s = prev[s]
    path.reverse()
    return path


# ----- 区間生成(状態列 -> Straight/Slalom) -----

@dataclass
class _Move:
    """状態列を「直進 run」と「ターン」に畳んだ中間表現。"""
    kind: str          # "straight" または "turn"
    cells: int = 0     # straight: 直進セル数
    turn_steps: int = 0  # turn: +1=右, -1=左, 2=180°


def _states_to_moves(path: List[State], start_dir: Direction) -> List[_Move]:
    """状態列を、進行方向の変化を境にした直進 run とターンの列へ。"""
    if len(path) < 2:
        return []
    # 各ステップの進行方向
    dirs: List[Direction] = [path[i + 1][2] for i in range(len(path) - 1)]
    moves: List[_Move] = []
    prev_dir = start_dir
    run = 0
    for d in dirs:
        ts = turn_between(prev_dir, d)
        if ts != 0:
            if run > 0:
                moves.append(_Move("straight", cells=run))
                run = 0
            moves.append(_Move("turn", turn_steps=ts))
        run += 1
        prev_dir = d
    if run > 0:
        moves.append(_Move("straight", cells=run))
    return moves


def moves_to_segments(
    moves: List[_Move], cfg: PlannerConfig
) -> List[Segment]:
    """中間表現 _Move 列を pattern.Straight/Slalom へ変換する。

    90°スラロームは前後の直進を旋回半径 R 分だけ短縮する(接線長=R)。
    直進の始端・終端速度は、隣にターンがあればスラローム速度、無ければ
    0(発進・停止)にする。180°は同方向90°スラローム×2で表す。
    """
    R = cfg.slalom_radius_mm
    v_cruise = cfg.straight_cruise_mmps
    v_slalom = cfg.slalom_mmps
    segs: List[Segment] = []

    n = len(moves)
    for i, mv in enumerate(moves):
        if mv.kind == "straight":
            prev_is_turn = i > 0 and moves[i - 1].kind == "turn"
            next_is_turn = i + 1 < n and moves[i + 1].kind == "turn"
            dist = mv.cells * CELL_MM
            if prev_is_turn:
                dist -= R
            if next_is_turn:
                dist -= R
            if dist < 0.0:
                dist = 0.0
            v_start = v_slalom if prev_is_turn else 0.0
            v_end = v_slalom if next_is_turn else 0.0
            # 直進が短すぎて巡航に届かない場合でも台形プロファイルが吸収する。
            if dist > 0.0:
                segs.append(
                    Straight(
                        distance_mm=dist,
                        v_start_mmps=v_start,
                        v_cruise_mmps=v_cruise,
                        v_end_mmps=v_end,
                    )
                )
        else:  # turn
            if mv.turn_steps == 2:
                # 180°: 直前の直進の終端速度に合わせ、同方向90°×2。
                # 回る向きは任意だが右回り(R)で統一。
                for _ in range(2):
                    segs.append(
                        Slalom(v_mmps=v_slalom, dir="R", radius_mm=R, angle_deg=90.0)
                    )
            else:
                segs.append(
                    Slalom(
                        v_mmps=v_slalom,
                        dir=slalom_dir_symbol(mv.turn_steps),
                        radius_mm=R,
                        angle_deg=90.0,
                    )
                )
    return segs


def plan(
    wm: WallMap,
    start: Tuple[int, int],
    start_dir: Direction,
    goal: Tuple[int, int],
    cfg: Optional[PlannerConfig] = None,
) -> List[Segment]:
    """壁マップ・現在位置姿勢・ゴールから走行区間列を生成する高水準API。"""
    cfg = cfg or PlannerConfig()
    path = find_path(wm, start, start_dir, goal, cfg)
    moves = _states_to_moves(path, start_dir)
    return moves_to_segments(moves, cfg)
