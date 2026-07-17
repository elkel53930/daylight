"""迷路データ構造(ハードウェア非依存)。

座標系: スタート (0, 0) が左下。x は東へ、y は北へ増える。
壁は UNKNOWN / OPEN / WALL の3状態で管理し、隣接セル間の整合は
Maze.set_wall() が常に両面を同時更新することで保証する。

Twilight の micromouse_algorithms.py の known/observed 2面持ちを
3状態 enum に再設計したもの(アルゴリズムは同等)。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, List, Optional, Sequence, Tuple

INF: int = 10 ** 9  # 到達不可能を表す距離


class Direction(IntEnum):
    """迷路座標系の絶対方位。"""

    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def left(self) -> "Direction":
        return Direction((int(self) + 3) % 4)

    def right(self) -> "Direction":
        return Direction((int(self) + 1) % 4)

    def back(self) -> "Direction":
        return Direction((int(self) + 2) % 4)

    @property
    def vector(self) -> Tuple[int, int]:
        return _DIR_VECTORS[self]


_DIR_VECTORS = {
    Direction.NORTH: (0, 1),
    Direction.EAST: (1, 0),
    Direction.SOUTH: (0, -1),
    Direction.WEST: (-1, 0),
}


class WallState(IntEnum):
    UNKNOWN = 0
    OPEN = 1
    WALL = 2


@dataclass
class Pose:
    """迷路座標系での位置と向き。"""

    x: int = 0
    y: int = 0
    heading: Direction = Direction.NORTH


@dataclass(frozen=True)
class GoalRegion:
    """ゴール領域(両端を含む矩形)。単一の定義として全ゴール判定に使う。"""

    x_min: int
    x_max: int
    y_min: int
    y_max: int

    def __post_init__(self) -> None:
        if self.x_min > self.x_max or self.y_min > self.y_max:
            raise ValueError(f"invalid goal region: {self}")

    def contains(self, x: int, y: int) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def cells(self) -> List[Tuple[int, int]]:
        return [
            (x, y)
            for y in range(self.y_min, self.y_max + 1)
            for x in range(self.x_min, self.x_max + 1)
        ]

    @classmethod
    def center_2x2(cls, maze_size: int) -> "GoalRegion":
        """クラシック競技の中央 2x2 ゴール。"""
        c = maze_size // 2
        return cls(x_min=c - 1, x_max=c, y_min=c - 1, y_max=c)


class Maze:
    """壁・訪問状態を保持する迷路。

    生成時に外周壁を WALL、内部壁を UNKNOWN で初期化する。
    """

    def __init__(self, size: int = 16):
        if size <= 0:
            raise ValueError("maze size must be positive")
        self.size = size
        # walls[y][x][dir] = WallState
        self._walls: List[List[List[WallState]]] = [
            [[WallState.UNKNOWN] * 4 for _ in range(size)] for _ in range(size)
        ]
        self._visited: List[List[bool]] = [[False] * size for _ in range(size)]

        for x in range(size):
            self.set_wall(x, 0, Direction.SOUTH, WallState.WALL)
            self.set_wall(x, size - 1, Direction.NORTH, WallState.WALL)
        for y in range(size):
            self.set_wall(0, y, Direction.WEST, WallState.WALL)
            self.set_wall(size - 1, y, Direction.EAST, WallState.WALL)

    # ---- 基本操作 ----

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.size and 0 <= y < self.size

    def _check_bounds(self, x: int, y: int) -> None:
        if not self.in_bounds(x, y):
            raise ValueError(f"cell out of bounds: {(x, y)}")

    def wall(self, x: int, y: int, d: Direction) -> WallState:
        self._check_bounds(x, y)
        return self._walls[y][x][int(d)]

    def set_wall(self, x: int, y: int, d: Direction, state: WallState) -> None:
        """壁を設定する。隣接セルの対面も必ず同時に更新する。

        外周(迷路の外との境界)を OPEN にすることはできない。
        """
        self._check_bounds(x, y)
        dx, dy = d.vector
        nx, ny = x + dx, y + dy
        if not self.in_bounds(nx, ny) and state == WallState.OPEN:
            raise ValueError(f"cannot open boundary wall: {(x, y)} {d.name}")

        self._walls[y][x][int(d)] = state
        if self.in_bounds(nx, ny):
            self._walls[ny][nx][int(d.back())] = state

    def observe_wall(self, x: int, y: int, d: Direction, exists: bool) -> None:
        """センサ観測結果で壁を更新する(bool → WallState)。"""
        self.set_wall(x, y, d, WallState.WALL if exists else WallState.OPEN)

    def visited(self, x: int, y: int) -> bool:
        self._check_bounds(x, y)
        return self._visited[y][x]

    def mark_visited(self, x: int, y: int) -> None:
        self._check_bounds(x, y)
        self._visited[y][x] = True

    def visited_count(self) -> int:
        return sum(row.count(True) for row in self._visited)

    # ---- 探索・経路計算 ----

    def can_pass(self, x: int, y: int, d: Direction, *, unknown_as_open: bool) -> bool:
        """セル (x,y) から方位 d へ移動できるか。

        unknown_as_open=True  : 探索用(未知壁は通れると仮定する楽観)
        unknown_as_open=False : 最短走行用(未知壁は壁とみなす安全側)
        """
        self._check_bounds(x, y)
        dx, dy = d.vector
        if not self.in_bounds(x + dx, y + dy):
            return False
        w = self._walls[y][x][int(d)]
        if w == WallState.WALL:
            return False
        if w == WallState.UNKNOWN:
            return unknown_as_open
        return True

    def distance_map(
        self,
        targets: Sequence[Tuple[int, int]],
        *,
        unknown_as_open: bool,
    ) -> List[List[int]]:
        """複数ターゲットへの距離マップ(multi-source BFS / Flood Fill)。"""
        dist = [[INF] * self.size for _ in range(self.size)]
        q: deque[Tuple[int, int]] = deque()
        for gx, gy in targets:
            self._check_bounds(gx, gy)
            dist[gy][gx] = 0
            q.append((gx, gy))

        while q:
            x, y = q.popleft()
            nd = dist[y][x] + 1
            for d in Direction:
                if not self.can_pass(x, y, d, unknown_as_open=unknown_as_open):
                    continue
                dx, dy = d.vector
                nx, ny = x + dx, y + dy
                if dist[ny][nx] > nd:
                    dist[ny][nx] = nd
                    q.append((nx, ny))
        return dist

    # ---- 永続化 ----

    def to_dict(self) -> dict:
        return {
            "size": self.size,
            "walls": [
                [[int(w) for w in cell] for cell in row] for row in self._walls
            ],
            "visited": [list(row) for row in self._visited],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Maze":
        maze = cls(int(data["size"]))
        walls = data["walls"]
        visited = data["visited"]
        if len(walls) != maze.size or len(visited) != maze.size:
            raise ValueError("maze data size mismatch")
        for y in range(maze.size):
            for x in range(maze.size):
                for d in range(4):
                    maze._walls[y][x][d] = WallState(walls[y][x][d])
                maze._visited[y][x] = bool(visited[y][x])
        return maze

    # ---- デバッグ表示 ----

    def render_text(
        self,
        *,
        pose: Optional[Pose] = None,
        goal: Optional[GoalRegion] = None,
        next_cell: Optional[Tuple[int, int]] = None,
        distance: Optional[List[List[int]]] = None,
    ) -> str:
        """ASCII で迷路を描画する。

        '+---+' 形式。未知壁は '.'、確定した壁は '-'/'|'、開放は空白。
        セル内は S(スタート)/G(ゴール)/^>v<(現在姿勢)/*(次目標)/
        .(訪問済み)を表示。distance 指定時は距離を16進2桁で表示する。
        """
        heading_char = {
            Direction.NORTH: "^",
            Direction.EAST: ">",
            Direction.SOUTH: "v",
            Direction.WEST: "<",
        }
        cw = 2 if distance is not None else 1  # セル内文字幅

        def h_wall(x: int, y: int, d: Direction) -> str:
            w = self._walls[y][x][int(d)]
            if w == WallState.WALL:
                return "-" * (cw + 2)
            if w == WallState.UNKNOWN:
                return "." * (cw + 2)
            return " " * (cw + 2)

        def v_wall(x: int, y: int, d: Direction) -> str:
            w = self._walls[y][x][int(d)]
            if w == WallState.WALL:
                return "|"
            if w == WallState.UNKNOWN:
                return "."
            return " "

        def cell_text(x: int, y: int) -> str:
            if distance is not None:
                v = distance[y][x]
                return "IN" if v >= INF else f"{v & 0xFF:02X}"
            if pose is not None and (x, y) == (pose.x, pose.y):
                return heading_char[pose.heading]
            if next_cell is not None and (x, y) == next_cell:
                return "*"
            if goal is not None and goal.contains(x, y):
                return "G"
            if (x, y) == (0, 0):
                return "S"
            if self._visited[y][x]:
                return "."
            return " "

        lines: List[str] = []
        for y in range(self.size - 1, -1, -1):
            top = ["+"]
            for x in range(self.size):
                top.append(h_wall(x, y, Direction.NORTH))
                top.append("+")
            lines.append("   " + "".join(top))

            mid: List[str] = []
            for x in range(self.size):
                mid.append(v_wall(x, y, Direction.WEST))
                mid.append(f" {cell_text(x, y):>{cw}} ")
            mid.append(v_wall(self.size - 1, y, Direction.EAST))
            lines.append(f"{y:>2} " + "".join(mid))

        bottom = ["+"]
        for x in range(self.size):
            bottom.append(h_wall(x, 0, Direction.SOUTH))
            bottom.append("+")
        lines.append("   " + "".join(bottom))

        xlabel = ["   "]
        for x in range(self.size):
            xlabel.append(f" {x:>{cw}}  "[: cw + 4 - 1])
        lines.append("".join(xlabel))
        return "\n".join(lines)


def load_maze_text(text: str, size: int = 16) -> Tuple[Maze, Optional[Tuple[int, int]]]:
    """クラシック迷路の ASCII 形式(mm_maze_solver 互換)から Maze を作る。

    シミュレータの「真の迷路」読み込み用。全壁が確定状態になる。
    'G' セルがあれば座標を返す。

    Twilight の read_maze_from_text_file() と同じフォーマットを、
    行単位のシンプルなパーサで読み直したもの。
    """
    maze = Maze(size)
    lines = [ln.rstrip("\r\n") for ln in text.splitlines() if ln.strip()]
    if len(lines) != 2 * size + 1:
        raise ValueError(f"expected {2 * size + 1} lines, got {len(lines)}")

    goal: Optional[Tuple[int, int]] = None

    # まず全ての内部壁を OPEN で初期化し、'-'/'|' を WALL で上書きする
    for y in range(size):
        for x in range(size):
            for d in Direction:
                dx, dy = d.vector
                if maze.in_bounds(x + dx, y + dy):
                    maze.set_wall(x, y, d, WallState.OPEN)

    for row in range(size + 1):
        hline = lines[2 * row]
        y = size - 1 - row  # この横壁線の下にあるセルの y
        for col in range(size):
            ch_idx = 1 + col * 2
            if ch_idx < len(hline) and hline[ch_idx] == "-":
                if row == 0:
                    maze.set_wall(col, size - 1, Direction.NORTH, WallState.WALL)
                else:
                    maze.set_wall(col, y + 1, Direction.SOUTH, WallState.WALL)

        if row == size:
            break
        vline = lines[2 * row + 1]
        y_cell = size - 1 - row
        for col in range(size + 1):
            ch_idx = col * 2
            if ch_idx < len(vline) and vline[ch_idx] == "|":
                if col == size:
                    maze.set_wall(size - 1, y_cell, Direction.EAST, WallState.WALL)
                else:
                    maze.set_wall(col, y_cell, Direction.WEST, WallState.WALL)
        for col in range(size):
            ch_idx = 1 + col * 2
            if ch_idx < len(vline) and vline[ch_idx] == "G":
                goal = (col, y_cell)

    return maze, goal
