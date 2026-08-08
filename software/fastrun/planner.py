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

pattern_from_cells は、交互90°ターンが3つ以上続く「階段」区間を
「45°スラローム → 斜め直進 → 45°スラローム」のショートカットに置き換え
られる(verify_loop の斜め走行と同じ構造、MazeSolver2015 の loadFromPath と
同じ発想)。斜め直進はコリドー中心を通る45°直線で、壁クリアランスの壁
ゲート(wm 指定時)で安全性を確認する。
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
    kind: str          # "straight" または "turn" または "diag"(斜め直進)
    cells: int = 0     # straight: 直進セル数
    d: Optional[Direction] = None  # straight: 直進 run の向き(斜め距離の算出用)
    turn_steps: int = 0  # turn: +1..+3=右45..135°, -1..-3=左, 4=180°
    dist_override: Optional[float] = None  # straight/diag: 距離を直接指定(斜め化時)


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


# ----- 斜めショートカット(階段区間の対角化) -----

def _cell_center(cell: Tuple[int, int]) -> Tuple[float, float]:
    """セルの中心の絶対位置 [mm](原点=南西端の柱、1マス180mm)。"""
    return (90.0 + CELL_MM * cell[0], 90.0 + CELL_MM * cell[1])


def _dir_unit(d: Direction) -> Tuple[float, float]:
    """向きの単位ベクトル(コース座標系)。"""
    dx, dy = d.delta
    n = math.hypot(dx, dy)
    return (dx / n, dy / n)


def _find_staircases(moves: List[_Move]) -> List[Tuple[int, int]]:
    """moves 列から「交互90°ターンの階段」区間を探す。

    戻り値: [(entry_idx, last_turn_idx)]。entry_idx は階段直前の直進 run、
    last_turn_idx は最後のターン(その直後が出口 run の直進)。階段は k(≥2)個の
    交互する90°ターン(例: +2,-2,+2 / +2,-2)と、その間の1セル直進から成る
    (MazeSolver2015 の loadFromPath の交互ターン検出と同じ構造)。k=2 は単一の
    コーナー(R90→L90 で1マス分の横移動)、k≥3(奇数)はコーナーの斜めショート
    カットに対応する。
    """
    n = len(moves)
    out: List[Tuple[int, int]] = []
    i = 0
    while i < n:
        if not (moves[i].kind == "straight" and i + 1 < n
                and moves[i + 1].kind == "turn"
                and abs(moves[i + 1].turn_steps) == 2):
            i += 1
            continue
        last_sign = 1 if moves[i + 1].turn_steps > 0 else -1
        k = 1
        pos = i + 2
        while (pos + 1 < n and moves[pos].kind == "straight"
               and moves[pos].cells == 1 and moves[pos + 1].kind == "turn"
               and abs(moves[pos + 1].turn_steps) == 2
               and (1 if moves[pos + 1].turn_steps > 0 else -1) == -last_sign):
            k += 1
            last_sign = -last_sign
            pos += 2
        last_turn = i + 2 * k - 1
        if k < 2:
            i += 1
            continue
        if last_turn + 1 >= n or moves[last_turn + 1].kind != "straight":
            i += 1
            continue
        out.append((i, last_turn))
        i = last_turn + 2
    return out


def _dist_point_segment(px: float, py: float, x1: float, y1: float,
                        x2: float, y2: float) -> float:
    """点から線分までの距離 [mm](diag_sim.py と同一)。"""
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    l2 = vx * vx + vy * vy
    if l2 == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / l2))
    return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))


def _trace_segments(segs: Sequence[Segment], sx0: float, sy0: float,
                    theta0: float, sample_step: float = 3.0,
                    slalom_step_deg: float = 2.0) -> List[Tuple[float, float]]:
    """区間列を mob path_controller と同じ内部フレーム式で世界座標にトレース
    する(diag_sim.py の trace() と同一の式)。"""
    pts: List[Tuple[float, float]] = [(sx0, sy0)]
    cx, cy = math.cos(theta0), math.sin(theta0)
    sx, sy = -math.sin(theta0), math.cos(theta0)
    ix = iy = 0.0
    head = 0.0

    def push(ix: float, iy: float) -> None:
        pts.append((sx0 + cx * ix + sx * iy, sy0 + cy * ix + sy * iy))

    for seg in segs:
        if isinstance(seg, Straight):
            length = seg.distance_mm
            steps = max(2, int(length / sample_step))
            for k in range(1, steps + 1):
                p = length * k / steps
                push(ix + p * math.cos(head), iy + p * math.sin(head))
            ix += length * math.cos(head)
            iy += length * math.sin(head)
        else:  # Slalom
            dirv = -1.0 if seg.dir == "R" else 1.0
            phi_end = math.radians(seg.angle_deg)
            steps = max(8, int(phi_end / math.radians(slalom_step_deg)))
            for k in range(1, steps + 1):
                phi = phi_end * k / steps
                th = head + dirv * phi
                ix2 = ix + dirv * seg.radius_mm * (math.sin(th) - math.sin(head))
                iy2 = iy + dirv * seg.radius_mm * (math.cos(head) - math.cos(th))
                push(ix2, iy2)
            head += dirv * phi_end
            ix, iy = ix2, iy2
    return pts


def _wall_segments_mm(wm: WallMap) -> List[Tuple[float, float, float, float]]:
    """WallMap の壁を世界座標の線分 (x1,y1,x2,y2) 列に変換する。"""
    segs: List[Tuple[float, float, float, float]] = []
    seen = set()
    for x in range(wm.width):
        for y in range(wm.height):
            x0, y0 = CELL_MM * x, CELL_MM * y
            for d in (Direction.N, Direction.E, Direction.S, Direction.W):
                if not wm.has_wall(x, y, d):
                    continue
                if d == Direction.N:
                    seg = (x0, y0 + CELL_MM, x0 + CELL_MM, y0 + CELL_MM)
                elif d == Direction.E:
                    seg = (x0 + CELL_MM, y0, x0 + CELL_MM, y0 + CELL_MM)
                elif d == Direction.S:
                    seg = (x0, y0, x0 + CELL_MM, y0)
                else:
                    seg = (x0, y0, x0, y0 + CELL_MM)
                key = frozenset(((round(seg[0], 3), round(seg[1], 3)),
                                 (round(seg[2], 3), round(seg[3], 3))))
                if key not in seen:
                    seen.add(key)
                    segs.append(seg)
    return segs


def _min_clearance(pts: Sequence[Tuple[float, float]],
                   wall_segs: Sequence[Tuple[float, float, float, float]]
                   ) -> float:
    """軌跡点列と全壁の最小距離 [mm]。"""
    return min(min(_dist_point_segment(px, py, *w) for w in wall_segs)
               for px, py in pts)


def _build_shortcut(moves: List[_Move], cells: Sequence[Tuple[int, int]],
                    entry_idx: int, last_turn_idx: int, cfg: PlannerConfig,
                    wm: Optional[WallMap], min_clearance_mm: float
                    ) -> Optional[List[_Move]]:
    """階段区間を斜めショートカットの _Move 列へ変換する(壁ゲート付き)。

    入口 run(entry_idx の直進)と出口 run(最後のターンの直後)を残し、その間を
    「45°スラローム → 斜め直進 → 45°スラローム」に置き換える。diag_sim.py と
    同じ世界座標ジオメトリで、階段の角を結ぶ45°直線(コリドー中心を通る)を
    通る。前後の直進は接線長を全て織り込み済みの距離に短縮する(dist_override)。
    wm 指定時は最小クリアランスが min_clearance_mm 未満なら None(階段のまま)。
    """
    entry = moves[entry_idx]
    exit_s = moves[last_turn_idx + 1]
    turns = [moves[t].turn_steps for t in range(entry_idx + 1,
                                                last_turn_idx + 1, 2)]
    k = len(turns)
    D1, D2 = entry.d, exit_s.d
    s_sign = 1 if turns[0] > 0 else -1
    Dd = D1.turned(s_sign)  # 45°斜め方向
    m0 = sum(m.cells for m in moves[:entry_idx] if m.kind == "straight")
    n1, n2 = entry.cells, exit_s.cells
    A = _cell_center(cells[m0 + n1])          # 入口 run の最終セル中心
    B = _cell_center(cells[m0 + n1 + k])      # 出口 run の2番目セル中心(= Q2 の基準)
    R = cfg.slalom_radius_mm
    T = slalom_tangent_mm(R, 45.0)            # 45°スラロームの接線長
    u1, u2 = _dir_unit(D1), _dir_unit(D2)
    half = CELL_MM / 2.0                      # 角から半セル手前で45°直線へ
    Q1 = (A[0] - half * u1[0], A[1] - half * u1[1])
    Q2 = (B[0] - half * u2[0], B[1] - half * u2[1])
    vx, vy = Q2[0] - Q1[0], Q2[1] - Q1[1]
    udx, udy = _dir_unit(Dd)
    if abs(vx * udy - vy * udx) > 1e-6:       # 45°整合性(想定外なら階段のまま)
        return None
    diag_len = math.hypot(vx, vy) - 2.0 * T
    prev_t = (_turn_tangent_mm(cfg, moves[entry_idx - 1])
              if entry_idx > 0 and moves[entry_idx - 1].kind == "turn" else 0.0)
    next_t = (_turn_tangent_mm(cfg, moves[last_turn_idx + 2])
              if last_turn_idx + 2 < len(moves)
              and moves[last_turn_idx + 2].kind == "turn" else 0.0)
    s1 = n1 * CELL_MM - prev_t - (half + T)
    s2 = n2 * CELL_MM - next_t - (half + T)
    if min(s1, s2, diag_len) < 1e-6:
        return None
    sub = [
        _Move("straight", cells=n1, d=D1, dist_override=s1),
        _Move("turn", turn_steps=1 if turns[0] > 0 else -1),
        _Move("diag", d=Dd, dist_override=diag_len),
        _Move("turn", turn_steps=1 if turns[-1] > 0 else -1),
        _Move("straight", cells=n2, d=D2, dist_override=s2),
    ]
    if wm is not None:
        px = A[0] - (n1 * CELL_MM - prev_t) * u1[0]
        py = A[1] - (n1 * CELL_MM - prev_t) * u1[1]
        pts = _trace_segments(moves_to_segments(sub, cfg), px, py,
                              D1.heading_rad)
        if _min_clearance(pts, _wall_segments_mm(wm)) < min_clearance_mm:
            return None
    return sub


def _moves_with_diag(path: List[State], start_dir: Direction,
                     cfg: PlannerConfig, wm: Optional[WallMap] = None,
                     min_clearance_mm: float = 50.0) -> List[_Move]:
    """状態列から _Move 列を作り、階段区間を斜めショートカットに置き換える。"""
    moves = _states_to_moves(path, start_dir)
    cells = [(x, y) for (x, y, _) in path]
    for entry_idx, last_turn_idx in reversed(_find_staircases(moves)):
        sub = _build_shortcut(moves, cells, entry_idx, last_turn_idx, cfg,
                              wm, min_clearance_mm)
        if sub is not None:
            moves[entry_idx:last_turn_idx + 2] = sub
    return moves


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
        if mv.kind in ("straight", "diag"):
            prev_is_turn = i > 0 and moves[i - 1].kind == "turn"
            next_is_turn = i + 1 < n and moves[i + 1].kind == "turn"
            if mv.dist_override is not None:
                # 斜めショートカット: 前後の接線短縮・斜め距離を全て織り込み済み
                dist = mv.dist_override
            else:
                cell_mm = DIAG_CELL_MM if mv.d.is_diagonal else CELL_MM
                dist = mv.cells * cell_mm
                if prev_is_turn:
                    dist -= _turn_tangent_mm(cfg, moves[i - 1])
                if next_is_turn:
                    dist -= _turn_tangent_mm(cfg, moves[i + 1])
            if dist < 1e-6:
                dist = 0.0
            if mv.kind == "diag":
                v_start = v_end = v_slalom
            else:
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
    wm: Optional[WallMap] = None,
    diagonal: bool = True,
    min_clearance_mm: float = 50.0,
) -> List[Segment]:
    """マスの列(セル列)から走行区間列(Straight/Slalom)を生成する。

    経路が「通るマスの列」として与えられる場合(将来のスタート→ゴール
    走行)に、そのまま PCLEAR/PADD/PRUN で走れる区間列へ変換する。
    隣接セル間の進行方向をセル中心間の向きとして状態列を作り、
    planner.py の既存区間生成(_states_to_moves + moves_to_segments)で
    Straight/Slalom へ展開する。スラロームの接線長短縮・45/90/135°の
    分割(180°=90°×2)・速度割当は plan() と同じロジック。

    交互90°ターンが3つ以上続く「階段」区間(diagonal=True、既定)は、現行
    verify_loop の斜め走行と同じく「45°スラローム → 斜め直進 → 45°
    スラローム」のショートカットに置き換える(MazeSolver2015 の
    loadFromPath と同じ発想)。wm を渡すとショートカット軌跡の壁最小
    クリアランスが min_clearance_mm 未満なら階段のまま残す。

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
    if diagonal:
        moves = _moves_with_diag(path, start_dir, cfg, wm, min_clearance_mm)
    else:
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
