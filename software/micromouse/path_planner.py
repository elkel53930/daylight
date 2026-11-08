"""最短経路計画(ハードウェア非依存)。

探索で得た迷路情報からセル経路を計算し、走行命令列へ変換する。
Twilight には存在しなかった機能で、Daylight で新規実装。

計画時は未知壁を壁とみなす(安全側)。探索でゴールと帰還を完走して
いれば、少なくとも実際に通った経路は既知なので経路は必ず存在する。

レイヤ分離:
    plan_cell_path()   : 迷路 → セル経路 (path planning)
    path_to_motions()  : セル経路 → 走行命令列 (motion planning)
命令列を実際の距離・速度プロファイルに落とすのは cell_runner.py。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from maze import Direction, INF, Maze, Pose


class MotionType(Enum):
    STRAIGHT = "STRAIGHT"      # n セル直進(セル中心 → セル中心)
    TURN_LEFT = "TURN_LEFT"    # その場 90 度左旋回
    TURN_RIGHT = "TURN_RIGHT"  # その場 90 度右旋回
    TURN_BACK = "TURN_BACK"    # その場 180 度旋回


@dataclass(frozen=True)
class Motion:
    type: MotionType
    cells: int = 0  # STRAIGHT のときのみ使用

    def __repr__(self) -> str:
        if self.type == MotionType.STRAIGHT:
            return f"STRAIGHT({self.cells})"
        return self.type.value


def plan_cell_path(
    maze: Maze,
    start: Tuple[int, int],
    goal_cells: Sequence[Tuple[int, int]],
    *,
    unknown_as_open: bool = False,
) -> Optional[List[Tuple[int, int]]]:
    """start から goal_cells のいずれかまでの最短セル経路を返す。

    ゴール側からの距離マップを作り、start から距離が 1 ずつ減る方向へ
    たどる。経路が無ければ None。
    """
    dist = maze.distance_map(goal_cells, unknown_as_open=unknown_as_open)
    x, y = start
    if dist[y][x] >= INF:
        return None

    path = [(x, y)]
    while dist[y][x] > 0:
        moved = False
        for d in Direction:
            if not maze.can_pass(x, y, d, unknown_as_open=unknown_as_open):
                continue
            dx, dy = d.vector
            nx, ny = x + dx, y + dy
            if dist[ny][nx] == dist[y][x] - 1:
                x, y = nx, ny
                path.append((x, y))
                moved = True
                break
        if not moved:  # 距離マップと can_pass が矛盾した場合(理論上起きない)
            return None
    return path


def path_to_motions(
    path: Sequence[Tuple[int, int]], start_heading: Direction
) -> Tuple[List[Motion], Direction]:
    """セル経路を走行命令列に変換する(連続直進は1命令に圧縮)。

    Returns:
        (命令列, 走行終了時の方位)
    """
    if len(path) < 2:
        return [], start_heading

    motions: List[Motion] = []
    heading = start_heading
    straight = 0

    def flush() -> None:
        nonlocal straight
        if straight > 0:
            motions.append(Motion(MotionType.STRAIGHT, straight))
            straight = 0

    for (x, y), (nx, ny) in zip(path, path[1:]):
        step = (nx - x, ny - y)
        target = next(d for d in Direction if d.vector == step)
        diff = (int(target) - int(heading)) % 4
        if diff == 1:
            flush()
            motions.append(Motion(MotionType.TURN_RIGHT))
        elif diff == 3:
            flush()
            motions.append(Motion(MotionType.TURN_LEFT))
        elif diff == 2:
            flush()
            motions.append(Motion(MotionType.TURN_BACK))
        heading = target
        straight += 1

    flush()
    return motions, heading
