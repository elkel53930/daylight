"""足立法による迷路探索(ハードウェア非依存)。

=== アルゴリズム: 足立法 (Flood Fill) ===
足立法は、ゴールから各セルまでの「距離」を逐次計算しながら探索する手法です。

1. ゴールセルからスタートして、全セルに「ゴールからの距離」を割り当てる
   (壁で隔てられたセルには無限大 INF を設定)
2. 現在位置の隣接セル(通行可能)のうち、距離が最小のセルへ移動
3. 新しい壁を発見するたびに距離マップを再計算
4. ゴールセルの距離が 0 になるまで繰り返す

この手法の利点:
- 迷路の全体像を知らなくてもゴールに到達できる
- 動的に壁情報を更新しながら経路を最適化できる
- 必ず最短経路を保証する(壁情報が完全になった場合)

=== モジュールの役割 ===
このモジュールは「次の移動方向を決める」純粋なロジックを担当します。
実際の移動は state_machine.py → cell_runner.py → mobile_base.py が担当します。

=== データフロー ===
  判断点(セル境界)で壁を観測
    ↓
  Explorer.update_walls() で迷路を更新
    ↓
  Maze.distance_map() で距離マップを再計算
    ↓
  Explorer.next_heading() で次の方向を決定
    ↓
  cell_runner がその方向に移動

=== 設計 ===
Twilight の AdachiExplorer から探索の意思決定のみを抽出したもの。
Twilight では壁更新・距離再計算・方位決定・自己位置更新が一つの
メソッドに同居していたが、テスト可能性のために Pose は呼び出し側
(状態機械)が所有し、Explorer は「迷路と現在位置から次の方位を
決める」ことだけを行う。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from maze import Direction, GoalRegion, INF, Maze, Pose


@dataclass(frozen=True)
class WallObservation:
    """判断点でのロボット相対の壁観測。None は「観測できない」。"""

    left: Optional[bool]
    front: Optional[bool]
    right: Optional[bool]


def relative_action(current: Direction, target: Direction) -> str:
    """現在の向きから目標方位へ向かうためのアクション名を返す。

    Returns: 'fwd' | 'right' | 'left' | 'back'
    """
    diff = (int(target) - int(current)) % 4
    return ("fwd", "right", "back", "left")[diff]


def observation_to_absolute(
    heading: Direction, obs: WallObservation
) -> Dict[Direction, bool]:
    """ロボットの向きに基づき、相対観測を絶対方位の壁有無に変換する。"""
    result: Dict[Direction, bool] = {}
    if obs.left is not None:
        result[heading.left()] = obs.left
    if obs.front is not None:
        result[heading] = obs.front
    if obs.right is not None:
        result[heading.right()] = obs.right
    return result


class Explorer:
    """足立法: ゴールへの距離マップを作り、距離が減る隣接セルへ進む。

    === アルゴリズム ===
    1. goal_cells から各セルへの最短距離を BFS で計算 (distance_map)
    2. 現在位置の隣接セル (通行可能) のうち、距離が最小のセルを選ぶ
    3. 壁を新しく発見するたびに距離マップを再計算
    
    === 設計 ===
    未知壁は通行可能とみなす(楽観)。壁の発見により距離マップは
    next_heading() のたびに再計算される。
    
    同距離のセルが複数ある場合は DIRECTION_PRIORITY の順で優先選択:
    front (直進) > left > right > back (後退)
    これにより、不要な旋回を減らす。
    """

    # 同距離の際の優先順位(直進優先で旋回回数を減らす)
    DIRECTION_PRIORITY = ("front", "left", "right", "back")

    def __init__(self, maze: Maze, goal_cells: Sequence[Tuple[int, int]]):
        """Explorer を生成する。
        
        Args:
            maze: 迷路データ構造
            goal_cells: ゴール領域のセル座標リスト
        """
        if not goal_cells:
            raise ValueError("goal_cells must not be empty")
        self.maze = maze
        self.goal_cells: List[Tuple[int, int]] = list(goal_cells)
        self.dist: List[List[int]] = maze.distance_map(
            self.goal_cells, unknown_as_open=True
        )

    def set_goals(self, goal_cells: Sequence[Tuple[int, int]]) -> None:
        """ゴール領域を変更し、距離マップを再計算する。"""
        if not goal_cells:
            raise ValueError("goal_cells must not be empty")
        self.goal_cells = list(goal_cells)
        self.recompute()

    def is_goal(self, x: int, y: int) -> bool:
        """指定セルがゴール領域内にあるか判定する。"""
        return (x, y) in self.goal_cells

    def update_walls(self, pose: Pose, obs: WallObservation) -> None:
        """観測した壁情報で迷路を更新する(隣接セル整合は Maze が保証)。"""
        for d, exists in observation_to_absolute(pose.heading, obs).items():
            self.maze.observe_wall(pose.x, pose.y, d, exists)

    def recompute(self) -> None:
        """距離マップを再計算する。"""
        self.dist = self.maze.distance_map(self.goal_cells, unknown_as_open=True)

    def next_heading(self, pose: Pose) -> Optional[Direction]:
        """現在位置から次に進むべき絶対方位を返す。

        事前条件:
        - pose が迷路の範囲内 (0 <= x < maze_size, 0 <= y < maze_size)
        - self.maze が有効な迷路データ構造
        
        事後条件:
        - 戻り値が Direction: その方向に進むべき (距離が最小の方向)
        - 戻り値が None: ゴールに到達不可能 (壁で囲まれている等)
        - self.dist が最新の距離マップに更新されている
        
        不変条件:
        - 同距離のセルが複数ある場合、DIRECTION_PRIORITY の順で優先選択
          (front > left > right > back)
        - 未知壁は通行可能と仮定 (unknown_as_open=True)
        """
        self.recompute()

        if self.dist[pose.y][pose.x] >= INF:
            return None

        rel_to_abs = {
            "front": pose.heading,
            "left": pose.heading.left(),
            "right": pose.heading.right(),
            "back": pose.heading.back(),
        }

        best: Optional[Direction] = None
        best_dist = INF
        for rel in self.DIRECTION_PRIORITY:
            d = rel_to_abs[rel]
            if not self.maze.can_pass(pose.x, pose.y, d, unknown_as_open=True):
                continue
            dx, dy = d.vector
            nd = self.dist[pose.y + dy][pose.x + dx]
            if nd < best_dist:
                best_dist = nd
                best = d
        return best
