"""セル単位走行(迷路座標系 → 連続座標系の変換層)。

=== 役割 ===
このモジュールは「迷路座標系」(セル単位)と「連続座標系」(mm/rad)の橋渡しをします。

state_machine.py はセル単位で「1セル進む」「右を向く」といった指示を出します。
cell_runner.py はそれを mm/rad の物理量に変換し、mobile_base.py に渡します。
mobile_base.py は JOGFWD/JOGTURN などのシリアルコマンドに変換して mob に送信します。

=== 壁センサフィードバック ===
探索走行・最短走行の両方で、壁センサフィードバック制御を有効にします。
これにより、機体が左右にずれた場合に壁センサ値(ls, rs)を読み取り、
path_controller が自動で機体を中央に寄せる補正を行います。

壁センサフィードバックを使用するメソッド:
- start_dash(): スタートから最初の判断点まで
- forward_one_cell(): 1セル前進
- exit_to_boundary(): センターから次の境界まで
- run_motion(STRAIGHT): 最短走行の直進区間

実装は mobile_base.forward_with_wall_feedback() を参照。
mob 側の path_controller が全セグメント完了時に DONE を送信するため、
Python 側は DONE を待つだけで完了を検出できます(2026-11-03変更)。

=== 重要な概念: 判断点 ===
探索走行では「セル境界」を判断点とします。
  - セル境界 = セルとセルの境界線 (0.5セル = 90mm の位置)
  - 判断点で壁センサを読み、進入先セルの壁の有無を判定する
  
セル中心 (1.0セル = 180mm) では旋回動作を行います。

=== 探索走行の幾何 ===
  - 直進: forward_with_wall_feedback(90mm) × 2 (境界 → 次の境界)
  - 旋回: forward_with_wall_feedback(90mm) → JOGTURN → forward_with_wall_feedback(90mm)
  - 停止: forward_with_wall_feedback(90mm) (中心で停止)

=== 最短走行の幾何 ===
  - 直進 n セル: forward_with_wall_feedback(n×180mm) (壁センサフィードバック付き)
  - 旋回: その場 JOGTURN

Twilight solve_maze_threaded.py の generate_action() の 90mm 定数を
セル寸法から導出するよう一般化したもの。
"""

from __future__ import annotations

import math
from typing import Callable, Optional

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
        壁センサフィードバックを有効にして直進する。
        
        事前条件:
        - WALL,1 で壁センサLEDが有効
        - config.start_offset_mm > 0
        
        事後条件:
        - self.base が start_offset_mm だけ前進している
        - 壁センサフィードバックにより左右位置が補正されている
        """
        # 壁センサフィードバック付きで前進
        if hasattr(self.base, 'forward_with_wall_feedback'):
            self.base.forward_with_wall_feedback(
                self.config.start_offset_mm,
                self.config.explore_speed_mmps,
                self.config.explore_accel_mmps2,
            )
        else:
            # シミュレータ用のフォールバック
            self.base.forward(
                self.config.explore_speed_mmps,
                self.config.explore_accel_mmps2,
                self.config.start_offset_mm,
            )

    def forward_one_cell(self) -> None:
        """境界から次の境界まで 1 セル前進する。

        半セルずつ 2 回に分けるのは Twilight 実績の踏襲: セル中心通過
        時点で完了を検出できるため、通信遅延があっても目標距離が
        半セル単位で確定する。
        壁センサフィードバックを有効にして直進する。
        
        事前条件:
        - WALL,1 で壁センサLEDが有効
        - self.base が判断点(セル境界)で停止中
        
        事後条件:
        - self.base が1セル分(180mm)前進している
        - 壁センサフィードバックにより左右位置が補正されている
        """
        half = self.config.half_cell_mm
        speed = self.config.explore_speed_mmps
        accel = self.config.explore_accel_mmps2
        
        # 壁センサフィードバック付きで前進
        if hasattr(self.base, 'forward_with_wall_feedback'):
            self.base.forward_with_wall_feedback(half, speed, accel)
            self.base.forward_with_wall_feedback(half, speed, accel)
        else:
            # シミュレータ用のフォールバック
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
        """セル中心から次の境界まで前進する(旋回後の再加速)。
        壁センサフィードバックを有効にして直進する。
        
        事前条件:
        - WALL,1 で壁センサLEDが有効
        - self.base がセル中心で停止中
        
        事後条件:
        - self.base が半セル分(90mm)前進している
        - 壁センサフィードバックにより左右位置が補正されている
        """
        if hasattr(self.base, 'forward_with_wall_feedback'):
            self.base.forward_with_wall_feedback(
                self.config.half_cell_mm,
                self.config.explore_speed_mmps,
                self.config.explore_accel_mmps2,
            )
        else:
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

    def explore_action(
        self, action: str, *, on_stopped: Optional[Callable[[], None]] = None
    ) -> None:
        """探索時の 1 アクション(判断点 → 次の判断点)。

        action: 'fwd' | 'left' | 'right' | 'back'

        on_stopped: 'fwd' 以外(旋回を伴うアクション)で、stop_at_center()
        により機体が実際に静止した直後・旋回前に呼ばれるコールバック。
        'fwd' は JOGFWD が距離到達で DONE を返すとその場を保持する
        (連続走行用)ため、機体が確実に静止するのはこのタイミングだけ
        であり、カメラ撮影やジャイロ再キャリブレーションなど静止を
        要する処理はここでのみ安全に行える。
        """
        if action == "fwd":
            self.forward_one_cell()
            return
        self.stop_at_center()
        if on_stopped is not None:
            on_stopped()
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
        """最短走行の1区間を実行する。
        
        事前条件:
        - motion が有効な Motion オブジェクト
        - STRAIGHT の場合、motion.cells > 0
        
        事後条件:
        - STRAIGHT: 壁センサフィードバック付きで指定距離前進
        - TURN_*: その場旋回
        
        不変条件:
        - STRAIGHT では forward_with_wall_feedback() を使用して
          壁センサフィードバック(path_wall_kp)を有効にする
        - シミュレータでは forward() にフォールバック
        """
        if motion.type == MotionType.STRAIGHT:
            if motion.cells <= 0:
                raise ValueError(f"invalid straight cells: {motion.cells}")
            distance = motion.cells * self.config.cell_size_mm
            speed = self.config.speed_run_speed_mmps
            accel = self.config.speed_run_accel_mmps2
            
            # 壁センサフィードバック付きで前進(2026-11-03変更)
            # 最短走行でも自己位置補正のため壁センサフィードバックを有効化
            if hasattr(self.base, 'forward_with_wall_feedback'):
                self.base.forward_with_wall_feedback(distance, speed, accel)
            else:
                # シミュレータ用のフォールバック
                self.base.stop_at(speed, accel, distance)
        elif motion.type == MotionType.TURN_LEFT:
            self.turn_left()
        elif motion.type == MotionType.TURN_RIGHT:
            self.turn_right()
        elif motion.type == MotionType.TURN_BACK:
            self.turn_back()
        else:
            raise ValueError(f"unknown motion: {motion}")
