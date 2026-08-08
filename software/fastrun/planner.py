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
最短距離ではなく最短時間(ターンが少なく直線的な経路)を選ぶ。8方向
(N/NE/E/SE/S/SW/W/NW)対応: 斜め移動は両隣接セルが開放なら角を切って
進める(wm.can_move が判定)とし、斜め1セルは √2×180mm として扱う。

区間生成では、スラロームが前後の直進を接線長(R·tan(θ/2)、90°なら R)
だけ「食う」ことを考慮して直進距離を短縮する。標準的なクラシックマウス
のスラローム区間割り。45/90/135°は単一スラローム、180°は90°×2。
"""

from __future__ import annotations

import heapq
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from geometry import (
    CELL_MM,
    DIAG_CELL_MM,
    Direction,
    direction_from_delta,
    slalom_angle_deg,
    slalom_dir_symbol,
    slalom_tangent_mm,
    turn_between,
)
from maze import WallMap

# pattern.py(software/mob/)の Straight/Slalom を再利用する。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mob"))
from pattern import Segment, Slalom, Straight  # noqa: E402


@dataclass(frozen=True)
class PlannerConfig:
    straight_cruise_mmps: float = 400.0   # 直進の巡航速度(ユーザー目標)
    slalom_mmps: float = 360.0            # スラローム旋回時の速度(R90で安全な値、実機で要調整)
    slalom_radius_mm: float = 90.0        # 90°スラローム半径(半セル)
    straight_accel_mmps2: float = 1000.0  # 直進の加減速度(mob params.path_accel と一致)
    turn_time_penalty_s: float = 0.35     # 経路探索用: 90°ターン1回の時間ペナルティ
    uturn_time_penalty_s: float = 0.9     # 経路探索用: 180°Uターンの時間ペナルティ


# ----- 経路探索 -----

State = Tuple[int, int, Direction]  # (x, y, 向き)


def _trapezoid_time_s(d_mm: float, v_mmps: float, v_start_mmps: float,
                      v_end_mmps: float, accel_mmps2: float) -> float:
    """台形速度プロファイルで距離 d_mm を v_start→v_cruise→v_end で走る
    所要時間[mob path_controller.advance_straight と同じ加減速則]。

    - 加速: v_start から v_cruise へ accel で
    - 減速: v_cruise から v_end へ accel で(残り距離が減速距離以下で開始)
    - 巡航: その間 v_cruise
    加速距離+減速距離が d を超える場合は巡航に届かない三角プロファイルになる
    (ピーク速度を d から逆算)。
    """
    def dist(v0: float, v1: float) -> float:
        return abs(v1 * v1 - v0 * v0) / (2.0 * accel_mmps2)

    d_acc = dist(v_start_mmps, v_mmps)
    d_dec = dist(v_mmps, v_end_mmps)
    t_acc = abs(v_mmps - v_start_mmps) / accel_mmps2
    t_dec = abs(v_mmps - v_end_mmps) / accel_mmps2

    if d_acc + d_dec <= d_mm:
        cruise = max(0.0, d_mm - d_acc - d_dec)
        return t_acc + cruise / v_mmps + t_dec

    # 三角プロファイル(巡航なし)。ピーク v_peak は
    #   d = (v_peak² - v_start²)/(2a) + (v_peak² - v_end²)/(2a)
    v_peak = math.sqrt((2.0 * accel_mmps2 * d_mm + v_start_mmps * v_start_mmps
                        + v_end_mmps * v_end_mmps) / 2.0)
    t1 = abs(v_peak - v_start_mmps) / accel_mmps2
    t2 = abs(v_peak - v_end_mmps) / accel_mmps2
    return t1 + t2


def _straight_time_s(cfg: PlannerConfig, dist_mm: float) -> float:
    """直進 dist_mm の実所要時間(発進0→巡航→停止0、ターン境界では0に落ちる前提)。"""
    return _trapezoid_time_s(dist_mm, cfg.straight_cruise_mmps, 0.0, 0.0,
                             cfg.straight_accel_mmps2)


def _edge_time(cfg: PlannerConfig, turn_steps: int,
               dist_mm: float = CELL_MM) -> float:
    """1セル進むエッジの所要時間の見積り(探索コスト)。

    直進は「0→巡航→0」の台形プロファイルで1セル(直交=CELL_MM / 斜め=
    DIAG_CELL_MM)走る時間(発進・停止の加減速を考慮)。ターンがあるエッジは
    それに角度に応じたスラローム区間の時間を加える(90°=turn_time_penalty、
    180°=uturn_time_penalty、45°=その半分)。
    """
    straight_t = _straight_time_s(cfg, dist_mm)
    if turn_steps == 0:
        return straight_t
    if abs(turn_steps) == 4:
        return straight_t + cfg.uturn_time_penalty_s
    return straight_t + cfg.turn_time_penalty_s * abs(turn_steps) / 2.0


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
            dist_mm = DIAG_CELL_MM if nd.is_diagonal else CELL_MM
            nd_cost = d + _edge_time(cfg, turn_steps, dist_mm)
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
    d: Optional[Direction] = None  # straight: 直進 run の向き(斜め距離の算出用)
    turn_steps: int = 0  # turn: +1..+3=右45..135°, -1..-3=左, 4=180°


def _states_to_moves(path: List[State], start_dir: Direction) -> List[_Move]:
    """状態列を、進行方向の変化を境にした直進 run とターンの列へ。"""
    if len(path) < 2:
        return []
    # 各ステップの進行方向
    dirs: List[Direction] = [path[i + 1][2] for i in range(len(path) - 1)]
    moves: List[_Move] = []
    prev_dir = start_dir
    run = 0
    run_dir: Optional[Direction] = None
    for d in dirs:
        ts = turn_between(prev_dir, d)
        if ts != 0:
            if run > 0:
                moves.append(_Move("straight", cells=run, d=run_dir))
                run = 0
            moves.append(_Move("turn", turn_steps=ts))
        run += 1
        run_dir = d
        prev_dir = d
    if run > 0:
        moves.append(_Move("straight", cells=run, d=run_dir))
    return moves


def _turn_tangent_mm(cfg: PlannerConfig, mv: _Move) -> float:
    """ターン _Move が前後の直進を食う接線長。

    180°は90°スラローム×2 なので各直進側に1個ぶん(R)だけ食われる。
    45/90/135°は R·tan(θ/2)。
    """
    if abs(mv.turn_steps) == 4:
        return cfg.slalom_radius_mm
    return slalom_tangent_mm(cfg.slalom_radius_mm, slalom_angle_deg(mv.turn_steps))


def moves_to_segments(
    moves: List[_Move], cfg: PlannerConfig
) -> List[Segment]:
    """中間表現 _Move 列を pattern.Straight/Slalom へ変換する。

    スラロームは前後の直進を接線長(R·tan(θ/2)、90°なら R)だけ短縮する。
    直進の始端・終端速度は、隣にターンがあればスラローム速度、無ければ
    0(発進・停止)にする。斜め直進 run の1セルは DIAG_CELL_MM。180°は
    同方向90°スラローム×2で表す。
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
            cell_mm = DIAG_CELL_MM if mv.d.is_diagonal else CELL_MM
            dist = mv.cells * cell_mm
            if prev_is_turn:
                dist -= _turn_tangent_mm(cfg, moves[i - 1])
            if next_is_turn:
                dist -= _turn_tangent_mm(cfg, moves[i + 1])
            if dist < 1e-6:
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
            if abs(mv.turn_steps) == 4:
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
                        angle_deg=slalom_angle_deg(mv.turn_steps),
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


def pattern_from_cells(
    cells: Sequence[Tuple[int, int]],
    start_dir: Direction,
    cfg: Optional[PlannerConfig] = None,
) -> List[Segment]:
    """マスの列(セル列)から走行区間列(Straight/Slalom)を生成する。

    経路が「通るマスの列」として与えられる場合(将来のスタート→ゴール
    走行)に、そのまま PCLEAR/PADD/PRUN で走れる区間列へ変換する。
    隣接セル間の進行方向をセル中心間の向きとして状態列を作り、
    planner.py の既存区間生成(_states_to_moves + moves_to_segments)で
    Straight/Slalom へ展開する。スラロームの接線長短縮・45/90/135°の
    分割(180°=90°×2)・速度割当は plan() と同じロジック。

    cells は隣接セルが連続する列(閉ループなら先頭と末尾が一致)。
    同一セルの連続(停止)や非隣接セル間は ValueError。
    """
    cfg = cfg or PlannerConfig()
    if len(cells) < 2:
        raise ValueError("cells は2つ以上のマスが必要")
    path: List[State] = [(cells[0][0], cells[0][1], start_dir)]
    for i in range(1, len(cells)):
        dx = cells[i][0] - cells[i - 1][0]
        dy = cells[i][1] - cells[i - 1][1]
        d = direction_from_delta(dx, dy)
        path.append((cells[i][0], cells[i][1], d))
    moves = _states_to_moves(path, start_dir)
    return moves_to_segments(moves, cfg)


def _slalom_time_s(seg: Slalom) -> float:
    """スラローム区間の所要時間(定速円弧)。"""
    arc_len_mm = seg.radius_mm * math.radians(seg.angle_deg)
    return arc_len_mm / seg.v_mmps


def estimate_time(segs: Sequence[Segment], cfg: Optional[PlannerConfig] = None) -> float:
    """区間列の実所要時間を見積もる(発進0・停止0の台形 + スラローム円弧)。

    mob の path_controller は Straight を発進0で開始し、区間境界で v_end/
    v_start を引き継ぐ。ここでは隣接するスラロームとの速度整合は無視し、
    各区間の時間の和として概算する(探索のための粗い見積もり用途)。
    """
    cfg = cfg or PlannerConfig()
    total = 0.0
    for seg in segs:
        if isinstance(seg, Straight):
            v_start = 0.0 if seg.v_start_mmps == 0.0 else seg.v_start_mmps
            v_end = 0.0 if seg.v_end_mmps == 0.0 else seg.v_end_mmps
            total += _trapezoid_time_s(seg.distance_mm, seg.v_cruise_mmps,
                                       v_start, v_end, cfg.straight_accel_mmps2)
        elif isinstance(seg, Slalom):
            total += _slalom_time_s(seg)
    return total
