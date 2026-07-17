"""セル単位走行(迷路座標系 → 連続座標系の変換層)。

「1セル進む」「右を向く」といった迷路座標系の動作を、mob の
FWD/STOP/TURN コマンド(mm / rad)へ変換する唯一のレイヤ。

探索走行の幾何(README「探索時のセルサイクル」参照):
- 判断点はセル境界。センサはそこで進入先セルの壁を読む。
- 直進:     FWD 半セル ×2       (境界 → 次の境界)
- 旋回:     STOP 半セル → TURN → FWD 半セル (中心で旋回)
- 停止:     STOP 半セル          (中心で停止)

最短走行はセル中心間で完結する:
- 直進 n セル: STOP n×セル長 (加速 → 巡航 → 減速停止)
- 旋回:        その場 TURN

Twilight solve_maze_threaded.py の generate_action() の 90mm 定数を
セル寸法から導出するよう一般化したもの。
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
        # base は mobile_base.MobileBase または simulator.SimMobileBase
        self.base = base
        self.config = config

    # ---- 探索走行(判断点 = セル境界) ----

    def start_dash(self) -> None:
        """スタート(セル中心)から最初の判断点まで前進する。

        セル中心とロボット設置位置のずれは start_offset_mm で調整する。
        """
        self.base.forward(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            self.config.start_offset_mm,
        )

    def forward_one_cell(self) -> None:
        """境界から次の境界まで 1 セル前進する。

        半セルずつ 2 回に分けるのは Twilight 実績の踏襲: セル中心通過
        時点で FWD の DONE が返るため、通信遅延があっても目標距離の
        累積(mob 側 cumulative_goal_dist)が半セル単位で確定する。
        """
        half = self.config.half_cell_mm
        speed = self.config.explore_speed_mmps
        accel = self.config.explore_accel_mmps2
        self.base.forward(speed, accel, half)
        self.base.forward(speed, accel, half)

    def stop_at_center(self) -> None:
        """境界からセル中心まで進んで停止する。"""
        self.base.stop_at(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            self.config.half_cell_mm,
        )

    def exit_to_boundary(self) -> None:
        """セル中心から次の境界まで前進する(旋回後の再加速)。"""
        self.base.forward(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            self.config.half_cell_mm,
        )

    def turn_left(self) -> None:
        self.base.turn(TURN_LEFT_RAD)

    def turn_right(self) -> None:
        self.base.turn(TURN_RIGHT_RAD)

    def turn_back(self) -> None:
        self.base.turn(TURN_BACK_RAD)

    def explore_action(self, action: str) -> None:
        """探索時の 1 アクション(判断点 → 次の判断点)。

        action: 'fwd' | 'left' | 'right' | 'back'
        """
        if action == "fwd":
            self.forward_one_cell()
            return
        self.stop_at_center()
        if action == "left":
            self.turn_left()
        elif action == "right":
            self.turn_right()
        elif action == "back":
            self.turn_back()
        else:
            raise ValueError(f"unknown action: {action}")
        self.exit_to_boundary()

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
