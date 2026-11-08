"""セル単位走行(迷路座標系 → 連続座標系の変換層).

探索走行の幾何:
- 判断点は区画中央から sensing_distance_mm (既定 90mm)
- 直進: FWD で判断位置まで → 壁判定 → FWD で残距離を走行(停止しない)
- 旋回: FWD で判断位置まで → 壁判定 → STOP で残距離を走行 → TURN

距離補正:
FWD の DONE 受信後、次の走行コマンド発行までに通信・処理遅延がある。
この間に進んだ距離を考慮し、コマンド発行直前のエンコーダ位置から
残距離を計算する。
"""

from __future__ import annotations

import math

from config import MicromouseConfig
from path_planner import Motion, MotionType

TURN_LEFT_RAD = math.pi / 2
TURN_RIGHT_RAD = -math.pi / 2
TURN_BACK_RAD = math.pi


class CellRunner:
    def __init__(self, base, config: MicromouseConfig):
        self.base = base
        self.config = config

    # ---- 探索走行 ----

    def advance_to_sensing_point(self) -> float:
        """判断位置まで FWD で前進する.

        Returns:
            FWD 完了後のエンコーダ距離[mm]
        """
        self.base.reset_distance()
        self.base.forward(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            self.config.sensing_distance_mm,
        )
        dist = self.base.read_distance()
        return dist if dist is not None else self.config.sensing_distance_mm

    def calculate_remaining_distance(self, actual_distance_mm: float) -> float:
        """次の区画中央までの残距離を計算する.

        Args:
            actual_distance_mm: コマンド発行直前の実測距離[mm]

        Returns:
            残距離[mm]. 0 以下の場合は 0 を返す.
        """
        remaining = self.config.cell_size_mm - actual_distance_mm
        return max(0.0, remaining)

    def continue_straight(self, remaining_mm: float) -> None:
        """直進継続: FWD で残距離を走行(停止しない)."""
        if remaining_mm <= 0:
            return
        self.base.forward(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            remaining_mm,
        )

    def stop_at_center(self, remaining_mm: float) -> None:
        """旋回のため区画中央で停止: STOP で残距離を走行."""
        if remaining_mm <= 0:
            return
        self.base.stop_at(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            remaining_mm,
        )

    def turn_left(self) -> None:
        self.base.turn(TURN_LEFT_RAD)

    def turn_right(self) -> None:
        self.base.turn(TURN_RIGHT_RAD)

    def turn_back(self) -> None:
        self.base.turn(TURN_BACK_RAD)

    # ---- 最短走行(セル中心 → セル中心) ----

    def run_motion(self, motion: Motion) -> None:
        if motion.type == MotionType.STRAIGHT:
            if motion.cells <= 0:
                raise ValueError(f"invalid straight cells: {motion.cells}")
            self.base.stop_at(
                self.config.speed_run_speed_mmps,
                self.config.speed_run_accel_mmps2,
                motion.cells * self.config.cell_size_mm,
            )
        elif motion.type == MotionType.TURN_LEFT:
            self.turn_left()
        elif motion.type == MotionType.TURN_RIGHT:
            self.turn_right()
        elif motion.type == MotionType.TURN_BACK:
            self.turn_back()
        else:
            raise ValueError(f"unknown motion: {motion}")
