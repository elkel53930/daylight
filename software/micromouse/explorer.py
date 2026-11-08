"""足立法による迷路探索(ハードウェア非依存)。

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

    未知壁は通行可能とみなす(楽観)。壁の発見により距離マップは
    next_heading() のたびに再計算される。
    """

    # 同距離の際の優先順位(直進優先で旋回回数を減らす)
    DIRECTION_PRIORITY = ("front", "left", "right", "back")

    def __init__(self, maze: Maze, goal_cells: Sequence[Tuple[int, int]]):
        if not goal_cells:
            raise ValueError("goal_cells must not be empty")
        self.maze = maze
        self.goal_cells: List[Tuple[int, int]] = list(goal_cells)
        self.dist: List[List[int]] = maze.distance_map(
            self.goal_cells, unknown_as_open=True
        )

    def set_goals(self, goal_cells: Sequence[Tuple[int, int]]) -> None:
        if not goal_cells:
            raise ValueError("goal_cells must not be empty")
        self.goal_cells = list(goal_cells)
        self.recompute()

    def is_goal(self, x: int, y: int) -> bool:
        return (x, y) in self.goal_cells

    def update_walls(self, pose: Pose, obs: WallObservation) -> None:
        """観測した壁情報で迷路を更新する(隣接セル整合は Maze が保証)。"""
        for d, exists in observation_to_absolute(pose.heading, obs).items():
            self.maze.observe_wall(pose.x, pose.y, d, exists)

    def recompute(self) -> None:
        self.dist = self.maze.distance_map(self.goal_cells, unknown_as_open=True)

    def next_heading(self, pose: Pose) -> Optional[Direction]:
        """現在位置から次に進むべき絶対方位を返す。

        距離マップを再計算した上で、優先順位(前→左→右→後)で
        最も距離の小さい通行可能な隣接セルを選ぶ。ゴールへ到達
        不可能な場合は None。
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
