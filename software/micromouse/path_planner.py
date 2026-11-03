"""最短経路計画(ハードウェア非依存)。

=== 概要 ===
探索で得た迷路情報から最短経路を計算し、走行命令列に変換します。

=== 処理フロー ===
1. plan_cell_path(): 迷路 → セル座標のリスト (例: [(0,0), (1,0), (1,1)])
2. path_to_motions(): セル経路 → 走行命令列 (例: [STRAIGHT(2), TURN_LEFT, STRAIGHT(3)])
3. cell_runner.py が命令列を実際の距離・速度に変換して mob に送信

=== 設計方針 ===
- 計画時は未知壁を壁とみなす(安全側)
- 探索でゴールと帰還を完走していれば、少なくとも実際に通った経路は既知なので経路は必ず存在する
- Twilight には存在しなかった機能で、Daylight で新規実装

=== MotionType の意味 ===
- STRAIGHT(n): n セル分直進 (連続する直進は1命令に圧縮)
- TURN_LEFT: 90度左旋回 (JOGTURN に変換)
- TURN_RIGHT: 90度右旋回 (JOGTURN に変換)
- TURN_BACK: 180度旋回 (JOGTURN に変換)

=== レイヤ分離 ===
このモジュールは「経路計画」のみを担当します。
命令列を実際の距離・速度プロファイルに落とすのは cell_runner.py の役割です。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from maze import Direction, INF, Maze, Pose


class MotionType(Enum):
    """走行命令のタイプ。
    
    STRAIGHT: n セル分直進する (連続する直進は1命令に圧縮される)
    TURN_LEFT: その場で90度左旋回 (反時計回り)
    TURN_RIGHT: その場で90度右旋回 (時計回り)
    TURN_BACK: その場で180度旋回
    """
    STRAIGHT = "STRAIGHT"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    TURN_BACK = "TURN_BACK"


@dataclass(frozen=True)
class Motion:
    """1つの走行命令を表す。
    
    type: 命令の種類 (MotionType)
    cells: STRAIGHTの場合の進行セル数。旋回命令では0。
    
    使用例:
      Motion(MotionType.STRAIGHT, 3)   # 3セル直進
      Motion(MotionType.TURN_LEFT)     # 90度左旋回
    """
    type: MotionType
    cells: int = 0

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

    事前条件:
    - start が迷路の範囲内
    - goal_cells が空でない
    - maze が有効な迷路データ構造
    
    事後条件:
    - 戻り値が List: start から goal までの最短経路のセル座標リスト
      path[0] == start, path[-1] in goal_cells
    - 戻り値が None: ゴールへの経路が存在しない (壁で囲まれている等)
    
    不変条件:
    - unknown_as_open=True  : 未知壁を通行可能と仮定 (探索モード)
    - unknown_as_open=False : 未知壁を壁と仮定 (最短走行モード)
    - 経路は連続するセルで構成される (隣接セルのみ)
    - 経路の各ステップでゴールへの距離が1ずつ減少する
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

    事前条件:
    - path が有効なセル経路 (連続する隣接セルのリスト)
    - start_heading が有効な Direction 値
    
    事後条件:
    - 戻り値の List[Motion]: 走行命令列
      連続する直進は1つの STRAIGHT 命令に圧縮される
    - 戻り値の Direction: 走行終了時の方位
    - motions を順に実行すると、path を辿れる
    
    不変条件:
    - STRAIGHT(n) は n セル分の直進を表す
    - TURN_LEFT/RIGHT/BACK はその場旋回を表す
    - 連続する同方向の直進は1つの STRAIGHT に圧縮される
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
