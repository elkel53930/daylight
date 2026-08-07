"""geometry.py — コース座標系と向きの定義(2026-08-03〜、2026-08-07 8方向化)。

コースは格子状の迷路。座標は (x, y) の整数セル座標で、俯瞰カメラの画像で
上を北とする(ユーザー指定)。すなわち:

    北(North) = +y 方向(画像の上)
    東(East)  = +x 方向(画像の右)
    南(South) = -y 方向(画像の下)
    西(West)  = -x 方向(画像の左)

向き(Direction)は8値(N/NE/E/SE/S/SW/W/NW、45°刻み・時計回り)。「右に1つ
回る(時計回り/CW)」= 値 +1 = 45°、「左に1つ回る(反時計回り/CCW)」= 値 -1
(mod 8)で表せる。turn_between / turned の回転ステップ数は全て 45° 単位。

スラローム旋回の方向記号は mob 側の規約に合わせる(pattern.py の Slalom):
CCW(左) = "L"、CW(右) = "R"。
"""

from __future__ import annotations

import math
from enum import IntEnum
from typing import Tuple

# クラシック迷路の1セル(区画)の一辺 [mm]。半径90mmスラロームが
# ちょうど半セル分の前進+半セル分の横移動になる寸法。
CELL_MM: float = 180.0

# 斜め1セル移動の距離(√2 × セル一辺)。座標間の直線距離。
DIAG_CELL_MM: float = CELL_MM * math.sqrt(2.0)


class Direction(IntEnum):
    N = 0
    NE = 1
    E = 2
    SE = 3
    S = 4
    SW = 5
    W = 6
    NW = 7

    @property
    def delta(self) -> Tuple[int, int]:
        """この向きへ1セル進んだときの (dx, dy)。北=+y に注意。"""
        return {
            Direction.N: (0, 1),
            Direction.NE: (1, 1),
            Direction.E: (1, 0),
            Direction.SE: (1, -1),
            Direction.S: (0, -1),
            Direction.SW: (-1, -1),
            Direction.W: (-1, 0),
            Direction.NW: (-1, 1),
        }[self]

    @property
    def is_diagonal(self) -> bool:
        """斜め(45°の倍数で軸と一致しない)向きか。"""
        return int(self) % 2 == 1

    def turned(self, steps: int) -> "Direction":
        """時計回り(右)に steps 回(45°単位)だけ回った向き。負なら反時計回り。"""
        return Direction((int(self) + steps) % 8)

    @property
    def heading_rad(self) -> float:
        """path_controller のワールド系での向き [rad] ではなく、コース系の
        参考角度。使う側では相対回転(turn_between)を使うので通常は不要。"""
        return {
            Direction.N: math.pi / 2,
            Direction.NE: math.pi / 4,
            Direction.E: 0.0,
            Direction.SE: -math.pi / 4,
            Direction.S: -math.pi / 2,
            Direction.SW: -3 * math.pi / 4,
            Direction.W: math.pi,
            Direction.NW: 3 * math.pi / 4,
        }[self]


def turn_between(frm: Direction, to: Direction) -> int:
    """frm から to への最小回転量を「時計回りステップ数(45°単位)」で返す。
    0=直進, +1=右45°, -1=左45°, +2=右90°, -2=左90°, +3=右135°, -3=左135°,
    4=180°(Uターン、符号は不定なので +4 で返す)。
    """
    diff = (int(to) - int(frm)) % 8
    if diff > 4:
        diff -= 8
    if diff == -4:
        return 4
    return diff  # 0..+4, -3..-1


def slalom_dir_symbol(turn_steps: int) -> str:
    """turn_between の結果(正=右/CW, 負=左/CCW)を pattern.Slalom の dir 記号へ。"""
    if turn_steps > 0:
        return "R"  # 時計回り/CW
    if turn_steps < 0:
        return "L"  # 反時計回り/CCW
    raise ValueError(f"0はターンでない: {turn_steps}")


def slalom_angle_deg(turn_steps: int) -> float:
    """|turn_steps|(<4) に対応するスラローム旋回角[deg](45°刻み)。"""
    return 45.0 * abs(turn_steps)


def slalom_tangent_mm(radius_mm: float, angle_deg: float) -> float:
    """スラローム円弧(半径 radius_mm・角度 angle_deg)が前後の直進を
    食う接線長 R·tan(θ/2)。90°ならちょうど R(従来モデルと一致)。"""
    return radius_mm * math.tan(math.radians(angle_deg) / 2.0)
