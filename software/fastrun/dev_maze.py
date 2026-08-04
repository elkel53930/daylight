"""dev_maze.py — 開発用4×4迷路の既知壁マップ(2026-08-04)。

RoboSweep 本番では迷路情報は Eiffel(俯瞰視覚エージェント)から既知として
与えられる。開発中は物理環境の4×4迷路を俯瞰カメラ(C270、Eiffel代理)から
起こしてここに固定する。壁は俯瞰画像 scratchpad/maze_map.jpg を各グリッド辺の
明度で自動判定して得た(内壁7本、全て max明度>60 で明瞭)。

座標系: 原点(0,0)=南西隅、x=東・y=北。セル(cx,cy)、(0,0)=南西、(3,3)=北東。
迷路が張り替えられたら再取得すること(build_dev_maze を作り直す)。
"""
from __future__ import annotations

from geometry import Direction
from maze import WallMap

# 俯瞰(2026-08-04)で検出した内壁。各タプル (cx, cy, 方向)。壁は共有なので
# 片側だけ立てれば WallMap が反対側にも立てる。
_DEV_WALLS = [
    (0, 3, Direction.S),  # (0,3)と(0,2)の境界(top-left 横壁)
    (2, 3, Direction.S),  # (2,3)と(2,2)の境界(top-middle 横バー)
    (2, 2, Direction.S),  # (2,2)と(2,1)の境界(middle 横バー)
    (1, 1, Direction.S),  # (1,1)と(1,0)の境界(bottom-left 横壁)
    (1, 1, Direction.W),  # (1,1)と(0,1)の境界(縦壁)
    (1, 0, Direction.W),  # (1,0)と(0,0)の境界(縦壁)
    (2, 1, Direction.E),  # (2,1)と(3,1)の境界(lower-L の右縦壁)
]


def build_dev_maze() -> WallMap:
    """開発用4×4迷路の既知壁マップを返す。"""
    wm = WallMap(4, 4)
    for cx, cy, d in _DEV_WALLS:
        wm.add_wall(cx, cy, d)
    return wm


def render_ascii(wm: WallMap) -> str:
    """WallMap を from_ascii と同形式のASCIIに描く(目視確認用)。上=北。"""
    w, h = wm.width, wm.height
    lines = []
    for gy in range(h, -1, -1):  # 上(北, y=h境界)から下へ
        # 格子(横壁)行
        row = "+"
        for cx in range(w):
            below = gy - 1  # この格子線の南側セル cy
            wall = wm.has_wall(cx, below, Direction.N) if 0 <= below < h else True
            row += "---+" if wall else "   +"
        lines.append(row)
        if gy == 0:
            break
        # セル(縦壁)行
        cy = gy - 1
        row = ""
        for cx in range(w):
            wwall = wm.has_wall(cx, cy, Direction.W)
            row += ("|" if wwall else " ") + "   "
        row += "|" if wm.has_wall(w - 1, cy, Direction.E) else " "
        lines.append(row)
    return "\n".join(lines)


if __name__ == "__main__":
    print(render_ascii(build_dev_maze()))
