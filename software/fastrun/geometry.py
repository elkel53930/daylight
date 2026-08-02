"""geometry.py — コース座標系と向きの定義(2026-08-03〜)。

コースは格子状の迷路。座標は (x, y) の整数セル座標で、俯瞰カメラの画像で
上を北とする(ユーザー指定)。すなわち:

    北(North) = +y 方向(画像の上)
    東(East)  = +x 方向(画像の右)
    南(South) = -y 方向(画像の下)
    西(West)  = -x 方向(画像の左)

向き(Direction)は N/E/S/W の4値。時計回りに N→E→S→W と並べており、
「右に1つ回る(時計回り/CW)」= 値 +1、「左に1つ回る(反時計回り/CCW)」
= 値 -1(mod 4)で表せる。

スラローム旋回の方向記号は mob 側の規約に合わせる(pattern.py の Slalom):
CCW(左) = "L"、CW(右) = "R"。
"""

from __future__ import annotations

from enum import IntEnum
from typing import Tuple

# クラシック迷路の1セル(区画)の一辺 [mm]。半径90mmスラロームが
# ちょうど半セル分の前進+半セル分の横移動になる寸法。
CELL_MM: float = 180.0


class Direction(IntEnum):
    N = 0
    E = 1
    S = 2
    W = 3

    @property
    def delta(self) -> Tuple[int, int]:
        """この向きへ1セル進んだときの (dx, dy)。北=+y に注意。"""
        return {
            Direction.N: (0, 1),
            Direction.E: (1, 0),
            Direction.S: (0, -1),
            Direction.W: (-1, 0),
        }[self]

    def turned(self, steps: int) -> "Direction":
        """時計回り(右)に steps 回だけ回った向き。負なら反時計回り(左)。"""
        return Direction((int(self) + steps) % 4)

    @property
    def heading_rad(self) -> float:
        """path_controller のワールド系での向き [rad] ではなく、コース系の
        参考角度。使う側では相対回転(turn_between)を使うので通常は不要。"""
        import math

        return {
            Direction.N: math.pi / 2,
            Direction.E: 0.0,
            Direction.S: -math.pi / 2,
            Direction.W: math.pi,
        }[self]


def turn_between(frm: Direction, to: Direction) -> int:
    """frm から to への最小回転量を「時計回りステップ数」で返す。
    0=直進, +1=右90°, -1=左90°, 2=180°(Uターン、符号は不定なので +2 で返す)。
    """
    diff = (int(to) - int(frm)) % 4
    if diff == 3:
        return -1
    return diff  # 0, 1, または 2


def slalom_dir_symbol(turn_steps: int) -> str:
    """turn_between の結果(+1=右, -1=左)を pattern.Slalom の dir 記号へ。"""
    if turn_steps == 1:
        return "R"  # 時計回り/CW
    if turn_steps == -1:
        return "L"  # 反時計回り/CCW
    raise ValueError(f"90°ターン以外は slalom_dir_symbol で扱えない: {turn_steps}")
