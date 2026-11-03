"""マイクロマウスのミッション状態機械。

=== アーキテクチャ ===
このモジュールはミッション全体のオーケストレーターとして機能します。

呼び出し階層:
  micromouse_app.py (UI/エントリポイント)
    ↓
  state_machine.py (このファイル - ミッション全体の制御)
    ↓
  cell_runner.py (セル座標 → 物理動作の変換)
    ↓
  mobile_base.py (シリアル通信 - mobへのコマンド送信)

=== 状態遷移 ===
    IDLE → CALIBRATION → MICROMOUSE_START → EXPLORATION → GOAL_REACHED
        → RETURN_TO_START → PATH_PLANNING → SPEED_RUN → FINISHED
    任意の状態 → ERROR / EMERGENCY_STOP

=== 各状態の役割 ===
- CALIBRATION: センサ初期化 (GCAL/RDST/RANG/WALL,1)
- EXPLORATION: 足立法でゴールまで探索走行
- RETURN_TO_START: スタート地点へ帰還 (経路情報収集のため)
- PATH_PLANNING: 既知の壁情報から最短経路を計算
- SPEED_RUN: 最短経路を高速走行

遷移条件の定義:
- 探索開始      : CALIBRATION 完了後、最初の JOGFWD 送信をもって開始
- ゴール到達    : 判断点で自セルが GoalRegion に含まれると判定した時
- ゴール後の停止: ゴール判定した判断点から半セル進んでセル中心に停止
- 最短経路計算  : スタート帰還完了後、未知壁=壁として BFS
- 最短走行開始  : 経路が存在し、confirm_speed_run() が True を返した時
- 異常時        : MOT,0,0 でモータ停止してから ERROR へ

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
    """探索 → 帰還 → 最短走行のミッション全体を実行する。
    
    === 不変条件 ===
    - self.base は常に有効な MobileBase/SimMobileBase インスタンス
    - self.maze は有効な迷路データ構造 (少なくとも外周壁が設定済み)
    - self.pose は常に迷路の範囲内 (0 <= x < maze_size, 0 <= y < maze_size)
    - self.state は MissionState のいずれかの値
    """

    def __init__(
        self,
        base,
        config: MicromouseConfig,
        *,
        observer: Optional[Callable[[MissionState, dict], None]] = None,
        log_fn: Optional[Callable[[dict], None]] = None,
        confirm_speed_run: Optional[Callable[[], bool]] = None,
    ):
        """ミッションを初期化する。
        
        事前条件:
        - base は MobileBase または SimMobileBase の有効なインスタンス
        - config は有効な MicromouseConfig インスタンス
        
        事後条件:
        - self.state == MissionState.IDLE
        - self.pose == Pose(0, 0, NORTH)  (スタートセル中心、北向き)
        - self.maze は外周壁が設定された初期状態
        """
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
        """状態を遷移させ、observer とログに通知する。
        
        事前条件:
        - state は MissionState の有効な値
        
        事後条件:
        - self.state == state
        - observer が設定されていれば呼び出されている
        - ログに記録されている (event="state")
        """
        self.state = state
        if self.observer is not None:
            self.observer(state, info)
        self._log(event="state", **info)

    def _log(self, **record) -> None:
        """ログを記録する。
        
        事前条件:
        - record は辞書形式のログデータ
        
        事後条件:
        - log_fn が設定されていれば、タイムスタンプ・状態・姿勢が追加された
          record が記録されている
        - log_fn が None の場合は何も起きない
        """
        if self.log_fn is None:
            return
        record.setdefault("t", time.time())
        record.setdefault("state", self.state.value)
        record.setdefault("pose", (self.pose.x, self.pose.y, self.pose.heading.name))
        self.log_fn(record)

    def _fail(self, message: str) -> None:
        """安全停止して MissionError を送出する。
        
        事前条件:
        - message はエラーメッセージ文字列
        
        事後条件:
        - モータは停止済み (emergency_stop 呼び出し済み)
        - self.error_message == message
        - MissionError が送出される (この関数は正常に戻らない)
        
        不変条件:
        - 呼び出し後、ミッションは ERROR 状態でなければならない
        """
        self.base.emergency_stop()
        self.error_message = message
        raise MissionError(message)

    def _read_sensors_checked(self) -> SensorFrame:
        """SEN をリトライ付きで取得し、バッテリー・センサ異常を検査する。
        
        事前条件:
        - モバイルベースが初期化済み
        
        事後条件:
        - 戻り値は有効な SensorFrame
        - バッテリー電圧 >= battery_min_v
        - センサ値が正常範囲内 (飽和・負値なし)
        
        例外:
        - MobileBaseError: センサ読み取りが連続失敗
        - MissionError: バッテリー低下またはセンサ異常 (モータ停止済み)
        """
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
    # 
    # 探索のメインループ (_explore_to):
    # 1. 判断点(セル境界)で停止
    # 2. センサを読み、壁情報を更新 (_observe_and_update)
    # 3. ゴールに到達したかチェック
    # 4. 次の移動方向を決定 (explorer.next_heading)
    # 5. cell_runner でその方向に移動
    # 6. 1. に戻る

    def _observe_and_update(self) -> Tuple[SensorFrame, WallObservation]:
        """判断点(セル境界)で壁センサを読み、迷路情報を更新する。
        
        事前条件:
        - self.pose が現在のセル(判断点)の座標を示している
        - モバイルベースが判断点で停止中
        
        事後条件:
        - self.maze に現在のセルの壁情報が更新されている
        - self.maze に現在セルの訪問済みフラグが立っている
        - 戻り値の WallObservation が現在の観測結果
        
        不変条件:
        - self.pose は変化しない (このメソッドは観測のみ)
        """
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
        """現在位置を指定方向に1セル進める。
        
        事前条件:
        - heading は有効な Direction 値
        - 進み先のセルが迷路の範囲内
        
        事後条件:
        - self.pose が1セル分進んだ新しい位置・向きを示している
        - self.pose.x == old_x + dx, self.pose.y == old_y + dy
        - self.pose.heading == heading
        
        不変条件:
        - self.pose は常に迷路の範囲内
        """
        dx, dy = heading.vector
        self.pose = Pose(self.pose.x + dx, self.pose.y + dy, heading)

    def _explore_to(self, goal_cells: List[Tuple[int, int]]) -> None:
        """現在の判断点から goal_cells のいずれかまで足立法で探索走行する。
        
        事前条件:
        - self.pose が現在の判断点(セル境界)の座標を示している
        - goal_cells が空でない
        - ゴールへの経路が存在する (未知壁は通行可能と仮定)
        
        事後条件:
        - self.pose がゴールセルの中心にある
        - self.pose.heading がゴールセルでの向き
        - self.maze に通過した全セルの壁情報と訪問記録が更新されている
        
        不変条件:
        - 各ステップでゴールへの距離が単調減少 (足立法の性質)
        - self.step_count が毎回インクリメント
        
        ループ不変条件:
        - self.pose は常に迷路の範囲内
        - self.maze は通過したセルの壁情報で更新済み
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
        """カメラでヨー角・前進距離を補正し、ジャイロを再キャリブレーションする。
        
        事前条件:
        - self.camera_corrector が初期化済み (None でない)
        - モバイルベースがセル中心で静止している (JOG到達後)
        - 前方に壁がある (カメラで壁上面の赤帯が検出できる状態)
        
        事後条件:
        - mob の角度・距離が補正されている (信頼できる推定の場合)
        - ジャイロが再キャリブレーションされている
        - self.cells_since_camera_correction == 0
        
        不変条件:
        - self.pose は変化しない (カメラ補正はセンサ値の補正のみ)
        - カメラ・GCALの失敗は探索を中止しない (補助的な保守処理)
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

        事前条件:
        - self.pose がセル中心の座標を示している
        - goal_cells が空でない
        - ゴールへの経路が存在する
        
        事後条件:
        - self.pose が次の判断点(境界)の座標を示している
        - モバイルベースが判断点に到達している
        
        不変条件:
        - is_start=True のとき、start_offset_mm の距離だけ前進
        - is_start=False のとき、half_cell_mm の距離だけ前進
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
        """静止状態でのセンサ初期化。全て DONE を確認して完了とする。
        
        事前条件:
        - モバイルベースが静止している (MOT,0,0 または IDLE 状態)
        - バッテリーが正常範囲内
        - センサが正常に動作している
        
        事後条件:
        - ジャイロがキャリブレーション済み (GCAL 完了)
        - 距離オドメトリがリセット済み (RDST 完了, odo_dist == 0)
        - 角度オドメトリがリセット済み (RANG 完了, odo_ang == 0)
        - 壁センサ LED が有効 (WALL,1)
        - センサ値が読み取り済み (バッテリー・正常性チェック済み)
        
        不変条件:
        - self.pose は変化しない (キャリブレーションはセンサ値の初期化のみ)
        """
        self.base.gyro_calibrate()
        self.base.reset_distance()
        self.base.reset_angle()
        self.base.wall_led(True)
        frame = self._read_sensors_checked()
        self._log(event="calibrated", vbatt=frame.vbatt)

    # ---- ミッション本体 ----
    # ミッション全体の実行フロー:
    # 1. CALIBRATION: センサ初期化 (ジャイロキャリブレーション、距離/角度リセット)
    # 2. MICROMOUSE_START: スタートセルの壁を観測
    # 3. EXPLORATION: 足立法でゴールまで探索走行
    # 4. GOAL_REACHED: ゴール到達
    # 5. RETURN_TO_START: スタート地点へ帰還 (追加の壁情報収集)
    # 6. PATH_PLANNING: 最短経路計算
    # 7. SPEED_RUN: 最短経路を高速走行

    def run(self) -> MissionState:
        """ミッションを実行し、最終状態を返す。

        事前条件:
        - self.base が初期化済み (シリアル接続確立 or シミュレータ生成済み)
        - ロボットがスタートセル (0,0) の中心に北向きで置かれている
        - 壁センサ LED が物理的に有効 (または WALL,1 で有効化できる)
        
        事後条件:
        - self.state が MissionState の最終状態 (FINISHED/ERROR/EMERGENCY_STOP)
        - モータが停止している (正常終了・異常終了いずれでも)
        - self.maze に探索済みの迷路データが格納されている
        - 走行ログが記録されている
        
        不変条件:
        - 例外発生時、必ず emergency_stop が呼ばれる
        - wall_led は必ず False にリセットされる
        - camera_corrector があれば必ず close() される
        
        状態遷移:
          CALIBRATION → MICROMOUSE_START → EXPLORATION → GOAL_REACHED
          → RETURN_TO_START → PATH_PLANNING → SPEED_RUN → FINISHED
          任意の状態 → ERROR / EMERGENCY_STOP
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
