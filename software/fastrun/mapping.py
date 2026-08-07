"""mapping.py — 壁センサ読みから壁マップを更新する純ロジック(2026-08-03〜)。

ロボットの前・左・右の壁センサ(機体相対)を、現在の向き(heading)に応じて
迷路の絶対方向(N/E/S/W)へ変換し WallMap に記録する。ハード非依存で
テスト可能にするため、シリアル等には依存しない。
"""

from __future__ import annotations

from typing import Tuple

from geometry import Direction
from maze import WallMap

# 壁あり/なしの判定しきい値(SEN実測: 壁あり ls/rs≈330・前≈190、開放≈0〜10)。
# #WEDGE検出のHIGH=200と揃える。
WALL_THRESHOLD = 200


def walls_from_raw(front_raw: int, left_raw: int, right_raw: int) -> Tuple[bool, bool, bool]:
    """生センサ値を壁あり(True)/なし(False)へ。"""
    return (
        front_raw > WALL_THRESHOLD,
        left_raw > WALL_THRESHOLD,
        right_raw > WALL_THRESHOLD,
    )


def update_walls(
    wm: WallMap,
    cell: Tuple[int, int],
    heading: Direction,
    front: bool,
    left: bool,
    right: bool,
) -> None:
    """機体相対(前/左/右)の壁有無を絶対方向へ変換して WallMap へ記録する。

    前 = heading 方向、左 = heading を反時計回りに90°、右 = 時計回りに90°。
    add_wall は隣接セルにも対称に壁を立てる。
    """
    x, y = cell
    if front:
        wm.add_wall(x, y, heading)
    if left:
        wm.add_wall(x, y, heading.turned(-2))
    if right:
        wm.add_wall(x, y, heading.turned(2))
