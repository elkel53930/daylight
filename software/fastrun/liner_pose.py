"""liner_pose.py — Liner の自己姿勢(セル座標+向き)と座標変換(2026-08-04)。

RoboSweep の Liner は「既知迷路で座標間を高速移動→マス中心へ壁上面補正」を
繰り返す。その過程で自分が今どのセルにどの向きで居るか(推定姿勢)を保持する。
オドメトリはドリフトするので、この姿勢は移動の意図(planner)と、到達後の
カメラ壁上面補正(絶対基準)で随時確定・再基準化する。

座標系(CLAUDE.md用語集): 原点(0,0)mm=南西隅の柱、x=東・y=北。セル(cx,cy)の
中心=絶対位置(90+180·cx, 90+180·cy)mm。初期座標(0,0)、北東端(3,3)。
向き(heading)は Direction(N/E/S/W)。ジャイロ/odo_ang/SANG 系の方位[deg]は
recenter 規約(北=0・西=+90(CCW正)・東=-90・南=±180)へ変換して使う。
"""
from __future__ import annotations

from dataclasses import dataclass

from geometry import CELL_MM, Direction

CELL_HALF_MM: float = CELL_MM / 2.0  # 90mm(マス中心のオフセット)

# Direction → ジャイロ/odo_ang/SANG 系の方位[deg]。recenter.HEADING_* と一致
# (北=0, 西=+90, 東=-90, 南=180)。TURN/SANG/reanchor はこの系の角度を使う。
# 斜めは中間値(北=0基準、CCW正): NE=-45, SE=-135, SW=+135, NW=+45。
_DIR_TO_GYRO_DEG = {
    Direction.N: 0.0,
    Direction.NE: -45.0,
    Direction.E: -90.0,
    Direction.SE: -135.0,
    Direction.S: 180.0,
    Direction.SW: 135.0,
    Direction.W: 90.0,
    Direction.NW: 45.0,
}


def direction_to_gyro_deg(d: Direction) -> float:
    """Direction を ジャイロ/SANG 系の絶対方位[deg]へ(北=0・西=+90・東=-90・南=180)。"""
    return _DIR_TO_GYRO_DEG[d]


def cell_center_mm(cx: int, cy: int) -> tuple[float, float]:
    """セル(cx,cy)の中心の絶対位置[mm](原点=南西隅、x東・y北)。"""
    return (CELL_HALF_MM + CELL_MM * cx, CELL_HALF_MM + CELL_MM * cy)


@dataclass(frozen=True)
class LinerPose:
    """Liner の推定姿勢: セル座標(cx,cy)+向き(heading)。"""
    cx: int
    cy: int
    heading: Direction

    @property
    def cell(self) -> tuple[int, int]:
        return (self.cx, self.cy)

    def center_mm(self) -> tuple[float, float]:
        """現在セル中心の絶対位置[mm]。"""
        return cell_center_mm(self.cx, self.cy)

    def heading_deg(self) -> float:
        """現在の向きの ジャイロ/SANG 系絶対方位[deg]。"""
        return direction_to_gyro_deg(self.heading)

    def moved(self, d: Direction, n: int = 1) -> "LinerPose":
        """向きは変えず、方位 d へ n セル進んだ姿勢を返す(範囲チェックはしない)。"""
        dx, dy = d.delta
        return LinerPose(self.cx + dx * n, self.cy + dy * n, self.heading)

    def facing(self, d: Direction) -> "LinerPose":
        """同じセルで向きだけ d にした姿勢を返す。"""
        return LinerPose(self.cx, self.cy, d)

    def advanced(self, n: int = 1) -> "LinerPose":
        """現在の向きへ n セル進んだ姿勢(向きはそのまま)。"""
        return self.moved(self.heading, n)
