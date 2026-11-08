"""マイクロマウスのミッション状態機械.

状態遷移:
    IDLE → CALIBRATION → RECENTERING → MICROMOUSE_START → EXPLORATION → FINISHED
    任意の状態 → ERROR / EMERGENCY_STOP

探索走行の距離補正:
    FWD の DONE 受信後、次のコマンド発行直前にエンコーダ位置を取得し、
    残距離を計算してから FWD/STOP を発行する。

カメラ位置補正:
    スタート区画で走り出す前に、カメラで前壁の赤エッジを検出し、
    ヨー角と前後位置を補正する。
"""

from __future__ import annotations

import time
from dataclasses import asdict
from enum import Enum
from typing import Callable, List, Optional, Tuple

from cell_runner import CellRunner
from config import MicromouseConfig
from explorer import Explorer, WallObservation, relative_action
from maze import Direction, Maze, Pose
from errors import AbortRequested, MobileBaseError
from path_planner import path_to_motions, plan_cell_path
from wall_detector import SensorFrame, WallDetector

try:
    from camera_recenter import recenter_start, Direction as RecenterDirection
    CAMERA_RECENTER_AVAILABLE = True
except (ImportError, AttributeError):
    CAMERA_RECENTER_AVAILABLE = False
    RecenterDirection = None


def walls_from_maze(maze, x, y):
    """maze から壁情報を取得する."""
    from maze import WallState, Direction as MazeDirection
    walls = {}
    for d, name in [(MazeDirection.NORTH, "north"), (MazeDirection.EAST, "east"),
                    (MazeDirection.SOUTH, "south"), (MazeDirection.WEST, "west")]:
        walls[name] = maze.wall(x, y, d) == WallState.WALL
    return walls


class MissionState(Enum):
    IDLE = "IDLE"
    CALIBRATION = "CALIBRATION"
    RECENTERING = "RECENTERING"
    MICROMOUSE_START = "MICROMOUSE_START"
    EXPLORATION = "EXPLORATION"
    GOAL_REACHED = "GOAL_REACHED"
    FINISHED = "FINISHED"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class MissionError(Exception):
    """ミッション続行不可能な異常."""


class MicromouseMission:
    """探索 → 帰還 → 最短走行のミッション全体を実行する."""

    def __init__(
        self,
        base,
        config: MicromouseConfig,
        *,
        observer: Optional[Callable[[MissionState, dict], None]] = None,
        log_fn: Optional[Callable[[dict], None]] = None,
        confirm_speed_run: Optional[Callable[[], bool]] = None,
    ):
        self.base = base
        self.config = config
        self.observer = observer
        self.log_fn = log_fn
        self.confirm_speed_run = confirm_speed_run

        self.detector = WallDetector(
            left_threshold=config.wall_left_threshold,
            right_threshold=config.wall_right_threshold,
            front_threshold=config.wall_front_threshold,
        )
        self.runner = CellRunner(base, config)
        self.maze = Maze(config.maze_size)
        self.goal = config.goal_region()
        self.explorer = Explorer(self.maze, self.goal.cells())
        self.pose = Pose(0, 0, Direction.NORTH)
        self.state = MissionState.IDLE
        self.step_count = 0
        self.error_message = ""

    def _set_state(self, state: MissionState, **info) -> None:
        self.state = state
        if self.observer is not None:
            self.observer(state, info)
        self._log(event="state", **info)

    def _log(self, **record) -> None:
        if self.log_fn is None:
            return
        record.setdefault("t", time.time())
        record.setdefault("state", self.state.value)
        record.setdefault("pose", (self.pose.x, self.pose.y, self.pose.heading.name))
        self.log_fn(record)

    def _fail(self, message: str) -> None:
        self.base.emergency_stop()
        self.error_message = message
        raise MissionError(message)

    def _read_sensors_checked(self) -> SensorFrame:
        frame: Optional[SensorFrame] = None
        for _ in range(self.config.sensor_retry):
            frame = self.base.read_sensors()
            if frame is not None:
                break
            time.sleep(0.1)
        if frame is None:
            raise MobileBaseError("sensor read failed")
        if frame.vbatt < self.config.battery_min_v:
            self._fail(f"battery low: {frame.vbatt:.2f} V")
        if not self.detector.is_sensor_sane(frame):
            self._fail(f"wall sensor out of range: {frame}")
        return frame

    def _observe_and_update(self) -> Tuple[SensorFrame, WallObservation]:
        """判断点でセンサを読み、迷路を更新する."""
        frame = self._read_sensors_checked()
        obs = self.detector.detect(frame)
        self.explorer.update_walls(self.pose, obs)
        self.maze.mark_visited(self.pose.x, self.pose.y)
        self._log(
            event="observe",
            sensors=asdict(frame),
            walls={"left": obs.left, "front": obs.front, "right": obs.right},
        )
        return frame, obs

    def _advance_pose(self, heading: Direction) -> None:
        dx, dy = heading.vector
        self.pose = Pose(self.pose.x + dx, self.pose.y + dy, heading)

    def _explore_one_cell(self, goal_cells: List[Tuple[int, int]], *, is_first: bool = False) -> Optional[Direction]:
        """1 セルの探索サイクルを実行する.

        呼び出し時点でロボットは判断位置にいる(is_first=True の場合は
        _resume_from_start で既に判断位置に移動済み)。

        1. 壁情報を取得
        2. 進行方向を決定
        3. 残距離を計算
        4. 直進 or 停止 + 旋回

        Returns:
            次のセルへ進む方向. ゴールに到達した場合は None.
        """
        self.step_count += 1
        if self.step_count > self.config.max_exploration_steps:
            self._fail(f"exceeded max steps: {self.config.max_exploration_steps}")

        # Phase 1: 壁情報を取得(既に判断位置にいる)
        try:
            frame, obs = self._observe_and_update()
        except MobileBaseError:
            self._fail("sensor read failed at sensing point")

        # Phase 2: ゴール判定
        if (self.pose.x, self.pose.y) in goal_cells:
            actual_dist = self.base.read_distance() or 0.0
            remaining = self.runner.calculate_remaining_distance(actual_dist)
            if remaining > 0:
                self.runner.stop_at_center(remaining)
            return None

        # Phase 3: 進行方向を決定
        heading = self.explorer.next_heading(self.pose)
        if heading is None:
            self._fail(f"no path to goal from ({self.pose.x}, {self.pose.y})")
        action = relative_action(self.pose.heading, heading)
        self._log(event="action", action=action, heading=heading.name)

        # Phase 4: 残距離を計算(距離補正)
        actual_dist = self.base.read_distance() or 0.0
        remaining = self.runner.calculate_remaining_distance(actual_dist)
        self._log(event="remaining_distance", actual_mm=actual_dist, remaining_mm=remaining)

        if remaining <= 0:
            self._fail(f"distance overrun: actual={actual_dist:.1f}mm")

        # Phase 5: 直進 or 停止 + 旋回
        if action == "fwd":
            self.runner.continue_straight(remaining)
        else:
            self.runner.stop_at_center(remaining)
            if action == "left":
                self.runner.turn_left()
            elif action == "right":
                self.runner.turn_right()
            elif action == "back":
                self.runner.turn_back()

        # Phase 6: 次のセルの判断位置まで前進
        if action == "fwd":
            self.runner.advance_to_sensing_point()
        else:
            self.runner.advance_to_sensing_point()

        return heading

    def _explore_to(self, goal_cells: List[Tuple[int, int]]) -> None:
        """goal_cells のいずれかに到達するまで探索走行を繰り返す."""
        self.explorer.set_goals(goal_cells)

        while True:
            next_heading = self._explore_one_cell(goal_cells)
            if next_heading is None:
                return
            self._advance_pose(next_heading)

    def _resume_from_center(self, goal_cells: List[Tuple[int, int]]) -> None:
        """セル中心から次のセルの判断位置まで移動する."""
        self.explorer.set_goals(goal_cells)
        heading = self.explorer.next_heading(self.pose)
        if heading is None:
            self._fail(f"no path to goal from ({self.pose.x}, {self.pose.y})")

        action = relative_action(self.pose.heading, heading)
        if action == "left":
            self.runner.turn_left()
        elif action == "right":
            self.runner.turn_right()
        elif action == "back":
            self.runner.turn_back()

        self.pose = Pose(self.pose.x, self.pose.y, heading)

        self.base.reset_distance()
        self.base.forward(
            self.config.explore_speed_mmps,
            self.config.explore_accel_mmps2,
            self.config.half_cell_mm,
        )

        dx, dy = heading.vector
        self.pose = Pose(self.pose.x + dx, self.pose.y + dy, heading)

    def _calibrate(self) -> None:
        self.base.gyro_calibrate()
        self.base.reset_distance()
        self.base.reset_angle()
        self.base.wall_led(True)
        frame = self._read_sensors_checked()
        self._log(event="calibrated", vbatt=frame.vbatt)

    def run(self) -> MissionState:
        """ミッションを実行し、最終状態を返す."""
        try:
            self._set_state(MissionState.CALIBRATION)
            self._calibrate()

            # カメラ位置補正(設定で有効かつモジュールが利用可能な場合)
            if self.config.camera_recenter_enabled and CAMERA_RECENTER_AVAILABLE:
                self._set_state(MissionState.RECENTERING)
                # スタート位置の壁情報を取得(外周壁は既知)
                walls = walls_from_maze(self.maze, 0, 0)
                current_heading = RecenterDirection[self.pose.heading.name]
                results = recenter_start(self.base, current_heading, walls, log_fn=self._log)
                for r in results:
                    self._log(
                        event="recenter_result",
                        axis=r.axis,
                        success=r.success,
                        message=r.message,
                        yaw_deg=r.yaw_deg,
                        offset_mm=r.offset_mm,
                    )
                    if not r.success:
                        self._log(event="recenter_warning", axis=r.axis, message=r.message)

            self._set_state(MissionState.MICROMOUSE_START)
            frame = self._read_sensors_checked()
            obs = self.detector.detect(frame)
            self.explorer.update_walls(self.pose, obs)
            self.maze.mark_visited(0, 0)
            self._log(
                event="start_observe",
                walls={"left": obs.left, "front": obs.front, "right": obs.right},
            )

            self._set_state(MissionState.EXPLORATION)
            self._resume_from_center(self.goal.cells())
            self._explore_to(self.goal.cells())
            self._set_state(
                MissionState.GOAL_REACHED, cell=(self.pose.x, self.pose.y)
            )

            self._set_state(MissionState.FINISHED, cell=(self.pose.x, self.pose.y))
            return self.state

        except AbortRequested:
            self.error_message = "aborted by user"
            self._set_state(MissionState.EMERGENCY_STOP)
            return self.state
        except MissionError as e:
            self._set_state(MissionState.ERROR, message=str(e))
            return self.state
        except (MobileBaseError, OSError) as e:
            self.base.emergency_stop()
            self.error_message = str(e)
            self._set_state(MissionState.ERROR, message=str(e))
            return self.state
        except Exception as e:
            self.base.emergency_stop()
            self.error_message = f"unexpected: {e}"
            self._set_state(MissionState.ERROR, message=str(e))
            raise
        finally:
            try:
                self.base.wall_led(False)
            except Exception:
                pass
