"""maze.py — 迷路の壁マップ(2026-08-03〜)。

各セル (x,y) の4方向(N/E/S/W)に壁があるか否かを保持する。壁は2つの
隣接セルで共有されるため、片方に壁を立てると反対側にも自動で立てる。
コース外周は常に壁とみなす(範囲外セルへは進めない)。

壁マップの入力は当面「明示的にプログラムから立てる」または「テキスト
表現から読む」の2通り。将来は俯瞰カメラや搭載カメラから自動生成する
(Phase 2以降)。
"""

from __future__ import annotations

from typing import Dict, Set, Tuple

from geometry import Direction


class WallMap:
    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        # (x, y) -> その隣に壁がある向きの集合
        self._walls: Dict[Tuple[int, int], Set[Direction]] = {}

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def add_wall(self, x: int, y: int, d: Direction) -> None:
        """セル (x,y) の d 方向に壁を立てる。隣接セルの反対側にも立てる。

        壁は直交辺(N/E/S/W)にのみ存在する。斜め方向へは立てられない。
        """
        if d.is_diagonal:
            raise ValueError(f"壁は直交辺にのみ立てられる: {d.name}")
        self._walls.setdefault((x, y), set()).add(d)
        dx, dy = d.delta
        nx, ny = x + dx, y + dy
        if self.in_bounds(nx, ny):
            self._walls.setdefault((nx, ny), set()).add(d.turned(4))

    def has_wall(self, x: int, y: int, d: Direction) -> bool:
        """セル (x,y) の d 方向に壁があるか。コース外周は常に True。斜め不可。"""
        if d.is_diagonal:
            raise ValueError(f"壁は直交辺にのみ存在する: {d.name}")
        dx, dy = d.delta
        if not self.in_bounds(x + dx, y + dy):
            return True
        return d in self._walls.get((x, y), set())

    def can_move(self, x: int, y: int, d: Direction) -> bool:
        """セル (x,y) から d 方向の隣セルへ進めるか(壁が無く範囲内)。

        斜め(dx,dy が共に非ゼロ)は角を切って通行するため、機体(幅≲90mm)が
        壁にクリップしないよう**角を挟む4辺の壁を全て確認**する。NE を例に
        すると (x,y)→(x+1,y+1) には次の4条件が全て必要:
          1. (x,y) の北壁なし((x,y)と(x,y+1)の境界)
          2. (x,y) の東壁なし((x,y)と(x+1,y)の境界)
          3. (x+1,y+1) の南壁なし((x+1,y+1)と(x+1,y)の境界)
          4. (x+1,y+1) の西壁なし((x+1,y+1)と(x,y+1)の境界)
        片側にでも壁があれば通れない。外周へ向かう斜めは範囲外判定で不可。
        """
        dx, dy = d.delta
        if dx != 0 and dy != 0:
            horiz = Direction.E if dx > 0 else Direction.W
            vert = Direction.N if dy > 0 else Direction.S
            nx, ny = x + dx, y + dy
            if not self.in_bounds(nx, ny):
                return False
            # 角を挟む4辺。turn(4) は180°(向きの反対)。
            return (
                self.can_move(x, y, horiz)
                and self.can_move(x, y, vert)
                and self.can_move(nx, ny, horiz.turned(4))
                and self.can_move(nx, ny, vert.turned(4))
            )
        return not self.has_wall(x, y, d)

    @classmethod
    def from_ascii(cls, text: str) -> "WallMap":
        """ASCIIアート表現から壁マップを作る(主にテスト用)。

        形式(3x3 の例、'+' 格子点・'-' 横壁・'|' 縦壁・空白は壁なし):

            +---+---+---+
            |           |
            +   +---+   +
            |   |       |
            +   +   +   +
            |   |       |
            +---+---+---+

        セル (0,0) は左下。行は上が y 最大。
        """
        lines = [ln for ln in text.splitlines() if ln.strip("")]
        # 空行を除かず、格子行(+を含む)とセル行が交互に来る前提で解析する。
        rows = [ln for ln in text.split("\n") if ln != ""]
        # 高さ・幅を格子から推定
        grid_rows = [r for r in rows if r.lstrip().startswith("+")]
        height = len(grid_rows) - 1
        width = (len(grid_rows[0].rstrip()) - 1) // 4
        wm = cls(width, height)
        # rows は上から: grid[0], cell[0], grid[1], cell[1], ... , grid[height]
        # y 座標は下が0なので、行インデックス r_top に対応する y は反転する。
        for gy in range(height + 1):
            grid_line = rows[gy * 2]
            # 横壁: セル (x, y) の上端/下端。gy=0 は最上段の格子線。
            y = height - gy  # この格子線の y 境界(上側セルの上端 = y, 下側セルの上端 = y-1 の上)
            for x in range(width):
                ch = grid_line[x * 4 + 1] if x * 4 + 1 < len(grid_line) else " "
                if ch == "-":
                    # この格子線の下のセル (x, y-1) の N 側に壁
                    if 0 <= y - 1 < height:
                        wm.add_wall(x, y - 1, Direction.N)
        for cy in range(height):
            cell_line = rows[cy * 2 + 1]
            y = height - 1 - cy
            for x in range(width + 1):
                ch = cell_line[x * 4] if x * 4 < len(cell_line) else " "
                if ch == "|":
                    # 縦壁: セル (x-1,y) の E 側 / セル (x,y) の W 側
                    if 0 <= x < width:
                        wm.add_wall(x, y, Direction.W)
                    elif x == width and width - 1 >= 0:
                        wm.add_wall(width - 1, y, Direction.E)
        return wm
