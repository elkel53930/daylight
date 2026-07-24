"""マイクロマウスのミッション状態機械。

状態遷移(README 参照):

    IDLE → CALIBRATION → MICROMOUSE_START → EXPLORATION → GOAL_REACHED
        → RETURN_TO_START → PATH_PLANNING → SPEED_RUN → FINISHED
    任意の状態 → ERROR / EMERGENCY_STOP

遷移条件の定義:
- 探索開始      : CALIBRATION 完了後、最初の FWD 送信をもって開始
- ゴール到達    : 判断点で自セルが GoalRegion に含まれると判定した時
- ゴール後の停止: ゴール判定した判断点から半セル進んでセル中心に停止
- 最短経路計算  : スタート帰還完了後、未知壁=壁として BFS
- 最短走行開始  : 経路が存在し、confirm_speed_run() が True を返した時
- 異常時        : QSTP → MOT,0,0 でモータ停止してから ERROR へ

このモジュールは UI に直接依存しない。進捗は observer コールバック、
中断要求は base 側の abort_check、走行ログは log_fn で外部へ渡す。
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


class MissionState(Enum):
    IDLE = "IDLE"
    CALIBRATION = "CALIBRATION"
    MICROMOUSE_START = "MICROMOUSE_START"
    EXPLORATION = "EXPLORATION"
    GOAL_REACHED = "GOAL_REACHED"
    RETURN_TO_START = "RETURN_TO_START"
    PATH_PLANNING = "PATH_PLANNING"
    SPEED_RUN = "SPEED_RUN"
    FINISHED = "FINISHED"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class MissionError(Exception):
    """ミッション続行不可能な異常(発生時点でモータ停止済みであること)。"""


class MicromouseMission:
    """探索 → 帰還 → 最短走行のミッション全体を実行する。"""

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

        # カメラによる位置補正(camera_correction.py)。実機(MobileBase)
        # にしか jog_turn 等が無いため、シミュレーションでは
        # config.camera_correction_enabled が true でも自動的に無効になる。
        self.camera_corrector = None
        if config.camera_correction_enabled and hasattr(base, "jog_turn"):
            from camera_correction import CameraCorrector

            self.camera_corrector = CameraCorrector()
        self.cells_since_camera_correction = 0

    # ---- 共通処理 ----

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
        """安全停止して MissionError を送出する。"""
        self.base.emergency_stop()
        self.error_message = message
        raise MissionError(message)

    def _read_sensors_checked(self) -> SensorFrame:
        """SEN をリトライ付きで取得し、バッテリー・センサ異常を検査する。"""
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

    # ---- 探索 ----

    def _observe_and_update(self) -> Tuple[SensorFrame, WallObservation]:
        """判断点でセンサを読み、迷路を更新する。"""
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

    def _explore_to(self, goal_cells: List[Tuple[int, int]]) -> None:
        """現在の判断点から goal_cells のいずれかまで足立法で探索走行する。

        戻るとき、ロボットは目標セルの中心に停止している。
        """
        self.explorer.set_goals(goal_cells)
        consecutive_failures = 0

        while True:
            self.step_count += 1
            if self.step_count > self.config.max_exploration_steps:
                self._fail(f"exceeded max steps: {self.config.max_exploration_steps}")

            try:
                _, obs = self._observe_and_update()
            except MobileBaseError:
                consecutive_failures += 1
                if consecutive_failures >= self.config.sensor_max_consecutive_failures:
                    self._fail("sensor failures exceeded limit")
                time.sleep(0.2)
                self.step_count -= 1
                continue
            consecutive_failures = 0

            # ゴール判定は GoalRegion 由来のセル集合との一致のみで行う
            if (self.pose.x, self.pose.y) in self.explorer.goal_cells:
                self.runner.stop_at_center()
                self._log(event="reached", cell=(self.pose.x, self.pose.y))
                return

            heading = self.explorer.next_heading(self.pose)
            if heading is None:
                self._fail(
                    f"no path to goal from ({self.pose.x}, {self.pose.y})"
                )
            action = relative_action(self.pose.heading, heading)
            self._log(event="action", action=action, heading=heading.name)

            should_correct = (
                self.camera_corrector is not None
                and obs.front
                and action != "fwd"
                and self.cells_since_camera_correction
                >= self.config.camera_correction_interval_cells
            )
            self.runner.explore_action(
                action,
                on_stopped=self._do_camera_correction if should_correct else None,
            )
            if should_correct:
                self.cells_since_camera_correction = 0
            else:
                self.cells_since_camera_correction += 1
            self._advance_pose(heading)

    def _do_camera_correction(self) -> None:
        """判断点で静止した直後に呼ばれる: カメラでヨー角・前進距離を
        補正し(信頼できる推定のみ、camera_correction.is_confident 参照)、
        補正の成否によらずジャイロを再キャリブレーションする。

        カメラ・GCALいずれも失敗しても探索は継続する(補助的な保守処理
        のため、失敗をミッション全体の失敗にはしない)。
        """
        estimate = None
        try:
            estimate = self.camera_corrector.try_correct(self.base)
        except Exception as e:
            self._log(event="camera_correction_error", error=str(e))
        self._log(
            event="camera_correction",
            applied=estimate is not None,
            estimate=asdict(estimate) if estimate is not None else None,
        )
        try:
            self.base.gyro_calibrate()
        except Exception as e:
            self._log(event="gyro_recalibrate_error", error=str(e))

    def _resume_from_center(
        self, goal_cells: List[Tuple[int, int]], *, is_start: bool = False
    ) -> None:
        """セル中心で停止した状態から探索を(再)開始し、次の判断点まで進む。

        is_start=True のときは中心→境界の距離に start_offset_mm を使う
        (設置位置の補正のため)。
        """
        self.explorer.set_goals(goal_cells)
        heading = self.explorer.next_heading(self.pose)
        if heading is None:
            self._fail(
                f"no path to goal from ({self.pose.x}, {self.pose.y})"
            )
        action = relative_action(self.pose.heading, heading)
        if action == "left":
            self.runner.turn_left()
        elif action == "right":
            self.runner.turn_right()
        elif action == "back":
            self.runner.turn_back()
        self.pose = Pose(self.pose.x, self.pose.y, heading)
        if is_start:
            self.runner.start_dash()
        else:
            self.runner.exit_to_boundary()
        self._advance_pose(heading)

    # ---- キャリブレーション ----

    def _calibrate(self) -> None:
        """静止状態でのセンサ初期化。全て DONE を確認して完了とする。"""
        self.base.gyro_calibrate()
        self.base.reset_distance()
        self.base.reset_angle()
        self.base.wall_led(True)
        frame = self._read_sensors_checked()
        self._log(event="calibrated", vbatt=frame.vbatt)

    # ---- ミッション本体 ----

    def run(self) -> MissionState:
        """ミッションを実行し、最終状態を返す。

        呼び出し時点でロボットはスタートセル (0,0) の中心に北向きで
        置かれていること。
        """
        try:
            self._set_state(MissionState.CALIBRATION)
            self._calibrate()

            self._set_state(MissionState.MICROMOUSE_START)
            # スタートセルの壁もハードコードせず、その場でセンサ観測する
            # (南・西は外周として初期化済み。北・東は迷路によって異なる)
            frame = self._read_sensors_checked()
            obs = self.detector.detect(frame)
            self.explorer.update_walls(self.pose, obs)
            self.maze.mark_visited(0, 0)
            self._log(
                event="start_observe",
                walls={"left": obs.left, "front": obs.front, "right": obs.right},
            )

            # 最初の前進コマンド送信をもって探索開始とする
            self._set_state(MissionState.EXPLORATION)
            self._resume_from_center(self.goal.cells(), is_start=True)
            self._explore_to(self.goal.cells())
            self._set_state(
                MissionState.GOAL_REACHED, cell=(self.pose.x, self.pose.y)
            )

            self._set_state(MissionState.RETURN_TO_START)
            self._resume_from_center([(0, 0)])
            self._explore_to([(0, 0)])

            self._set_state(MissionState.PATH_PLANNING)
            path = plan_cell_path(
                self.maze, (0, 0), self.goal.cells(), unknown_as_open=False
            )
            if path is None:
                # 帰還時に通った経路が既知のため理論上到達しないが防御
                self._fail("no safe path found after exploration")
            motions, _ = path_to_motions(path, self.pose.heading)
            self._log(event="plan", path=path, motions=[repr(m) for m in motions])

            if self.confirm_speed_run is not None and not self.confirm_speed_run():
                self._set_state(MissionState.FINISHED, note="speed run skipped")
                return self.state

            self._set_state(MissionState.SPEED_RUN, motions=len(motions))
            # 静止中に再校正してから走る
            self._calibrate()
            for motion in motions:
                self._log(event="motion", motion=repr(motion))
                self.runner.run_motion(motion)
            self.pose = Pose(path[-1][0], path[-1][1], self.pose.heading)

            self._set_state(MissionState.FINISHED, cell=path[-1])
            return self.state

        except AbortRequested:
            # base 側でモータ停止済み
            self.error_message = "aborted by user"
            self._set_state(MissionState.EMERGENCY_STOP)
            return self.state
        except MissionError as e:
            # _fail() 経由: モータ停止済み
            self._set_state(MissionState.ERROR, message=str(e))
            return self.state
        except (MobileBaseError, OSError) as e:
            self.base.emergency_stop()
            self.error_message = str(e)
            self._set_state(MissionState.ERROR, message=str(e))
            return self.state
        except Exception as e:
            # 予期しない例外でも必ず安全停止する
            self.base.emergency_stop()
            self.error_message = f"unexpected: {e}"
            self._set_state(MissionState.ERROR, message=str(e))
            raise
        finally:
            try:
                self.base.wall_led(False)
            except Exception:
                pass
            if self.camera_corrector is not None:
                self.camera_corrector.close()
