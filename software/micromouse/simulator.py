"""仮想 mob(シミュレータ)。

mobile_base.MobileBase と同じインターフェースを持ち、真の迷路
(load_maze_text で読んだ全壁確定の Maze)の上で走行コマンドを
シミュレートする。ハードウェア無しで状態機械〜探索アルゴリズムの
エンドツーエンド検証に使う。

- 位置は連続座標 (mm) で管理し、コマンドの距離をそのまま積算する。
- 移動が壁を突き抜ける場合は SimulationCrash を送出する
  (アルゴリズムの誤りを見逃さないため)。
- read_sensors() は「現在位置の少し先のセル」の壁から SEN 値を合成する。
  判断点(セル境界)では進入先セルの壁が読める。実機と同じ幾何。
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple

from errors import AbortRequested
from maze import Direction, Maze, WallState
from wall_detector import SensorFrame


class SimulationCrash(Exception):
    """壁への衝突(アルゴリズム誤りの検出)。"""


# 合成センサ値: しきい値(既定 100/100/50)を確実にまたぐ値
_SIDE_WALL_VALUE = 250
_FRONT_WALL_VALUE = 200
_NO_WALL_VALUE = 5


class SimMobileBase:
    def __init__(
        self,
        true_maze: Maze,
        *,
        cell_size_mm: float = 180.0,
        battery_v: float = 7.4,
        abort_check: Optional[Callable[[], bool]] = None,
        trace: Optional[List[str]] = None,
    ):
        self.maze = true_maze
        self.cell = cell_size_mm
        self.battery_v = battery_v
        self.abort_check = abort_check
        self.trace = trace  # 実行コマンドの記録(テスト検証用)

        # スタートセル (0,0) の中心、北向き
        self.x_mm = cell_size_mm / 2.0
        self.y_mm = cell_size_mm / 2.0
        self.heading = Direction.NORTH
        self.odo_dist_mm = 0.0
        self.odo_ang_rad = 0.0
        self.wall_led_enabled = False

    # ---- 内部 ----

    def _record(self, s: str) -> None:
        if self.trace is not None:
            self.trace.append(s)

    def _cell_of(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        return int(x_mm // self.cell), int(y_mm // self.cell)

    def _move(self, distance_mm: float) -> None:
        """heading 方向へ移動。横切るセル境界ごとに壁をチェックする。"""
        if self.abort_check is not None and self.abort_check():
            raise AbortRequested("aborted by user (sim)")

        dx, dy = self.heading.vector
        remaining = distance_mm
        step = 1.0  # 1mm 刻みで境界横断を検出(シンプルさ優先)
        while remaining > 1e-9:
            d = min(step, remaining)
            before = self._cell_of(
                self.x_mm + dx * 1e-6, self.y_mm + dy * 1e-6
            )
            self.x_mm += dx * d
            self.y_mm += dy * d
            after = self._cell_of(
                self.x_mm + dx * 1e-6, self.y_mm + dy * 1e-6
            )
            if before != after:
                bx, by = before
                if not self.maze.in_bounds(*after):
                    raise SimulationCrash(
                        f"drove out of maze at cell {before} heading {self.heading.name}"
                    )
                if self.maze.wall(bx, by, self.heading) == WallState.WALL:
                    raise SimulationCrash(
                        f"crashed into wall: cell {before} -> {after} "
                        f"({self.heading.name})"
                    )
            remaining -= d
        self.odo_dist_mm += distance_mm

    def _lookahead_cell(self) -> Optional[Tuple[int, int]]:
        """現在位置の少し先(判断点なら進入先)のセル。迷路外なら None。"""
        dx, dy = self.heading.vector
        cx, cy = self._cell_of(self.x_mm + dx * 1.0, self.y_mm + dy * 1.0)
        if not self.maze.in_bounds(cx, cy):
            return None
        return (cx, cy)

    # ---- MobileBase 互換インターフェース ----

    def close(self) -> None:
        pass

    def forward(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        self._record(f"FWD,{distance_mm:.0f}")
        self._move(distance_mm)

    def stop_at(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        self._record(f"STOP,{distance_mm:.0f}")
        self._move(distance_mm)

    def turn(self, angle_rad: float) -> None:
        self._record(f"TURN,{angle_rad:.4f}")
        quarter_turns = round(angle_rad / (math.pi / 2)) % 4
        # 正 = 左回り (CCW)
        for _ in range(quarter_turns):
            self.heading = self.heading.left()
        self.odo_ang_rad += angle_rad

    def quick_stop(self) -> float:
        self._record("QSTP")
        return 0.0

    def motors_off(self) -> None:
        self._record("MOT,0,0")

    def emergency_stop(self) -> None:
        self._record("ESTOP")

    def gyro_calibrate(self, timeout_s: float = 10.0) -> None:
        self._record("GCAL")

    def reset_distance(self) -> None:
        self._record("RDST")
        self.odo_dist_mm = 0.0

    def reset_angle(self) -> None:
        self._record("RANG")
        self.odo_ang_rad = 0.0

    def wall_led(self, enabled: bool) -> None:
        self.wall_led_enabled = enabled

    def read_sensors(self, timeout_s: float = 2.0) -> Optional[SensorFrame]:
        cell = self._lookahead_cell()
        if cell is None:
            lf = ls = rs = rf = _NO_WALL_VALUE
        else:
            cx, cy = cell
            h = self.heading

            def wall(d: Direction) -> bool:
                return self.maze.wall(cx, cy, d) == WallState.WALL

            ls = _SIDE_WALL_VALUE if wall(h.left()) else _NO_WALL_VALUE
            rs = _SIDE_WALL_VALUE if wall(h.right()) else _NO_WALL_VALUE
            front = _FRONT_WALL_VALUE if wall(h) else _NO_WALL_VALUE
            lf = rf = front  # Daylight: lf/rf は同じ前センサ値

        if not self.wall_led_enabled:
            lf = ls = rs = rf = 0  # LED 無効時は差分値が出ない(実機と同じ)

        return SensorFrame(
            gyro_radps=0.0,
            vbatt=self.battery_v,
            lf=lf,
            ls=ls,
            rs=rs,
            rf=rf,
            enc_r=0,
            enc_l=0,
            odo_dist_mm=self.odo_dist_mm,
            odo_ang_rad=self.odo_ang_rad,
            ball_raw=0,
            ball_det=False,
        )

    # ---- テスト用ヘルパ ----

    @property
    def current_cell(self) -> Tuple[int, int]:
        return self._cell_of(self.x_mm, self.y_mm)

    def is_at_center_of(self, cell: Tuple[int, int], tol_mm: float = 1.0) -> bool:
        cx = (cell[0] + 0.5) * self.cell
        cy = (cell[1] + 0.5) * self.cell
        return abs(self.x_mm - cx) <= tol_mm and abs(self.y_mm - cy) <= tol_mm
