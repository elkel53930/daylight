"""L1 ボタンで実行するボール回収シーケンス(ハード非依存)。

remote_controller.py から呼ばれる。テスト容易性のため、実際の MobileBase/
FutabaServo ではなく duck-typed な base/arm を受け取る:

    base.set_reload_servo(angle_deg)
    base.set_fan_percent(percent)
    base.read_sensors(timeout_s) -> フレーム(.ball_raw 属性を持つ) | None
    arm.set_angle(angle_deg, move_time_ms=0)

シーケンス(software/arm/ball_sequence.py の単体スクリプト版と同じ内容を
リモート操作の L1 ボタンに統合したもの):

    1. アームサーボ・リロードサーボを 0 度へ
    2. 0.5 秒待つ
    3. リロードサーボを 140 度へ
    4. アームサーボを 1000ms で 103 度へ
    5. アームサーボが 103 度に到達(1000ms後)したらファン Duty 50%
    6. ボールセンサ値(ball_raw)が 100 を3回連続で超えたら
       アームサーボを 1000ms で 0 度へ。
       100 を超えずに2秒経過したらリロードサーボを 0 度にして終了(失敗)。
    7. アームサーボが 0 度に到達(1000ms後)したらファン Duty 0%
    8. ボールセンサ値が 100 未満ならリロードサーボを 0 度へ(保持失敗)
    9. そうでなければボール保持成功
    10. 終了

sleep/now はテスト用の差し替え口(既定は実際の time.sleep/time.monotonic)。
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Protocol

ARM_HOME_DEG = 0.0
ARM_GRAB_DEG = 103.0
ARM_MOVE_TIME_MS = 1000

RELOAD_HOME_DEG = 0.0
RELOAD_SCOOP_DEG = 140.0
RELOAD_RELEASE_DEG = 180.0

FAN_ON_PERCENT = 50.0
FAN_OFF_PERCENT = 0.0

BALL_THRESHOLD = 100
BALL_CONSECUTIVE = 3
BALL_WAIT_TIMEOUT_S = 2.0
BALL_POLL_TIMEOUT_S = 0.3
BALL_FINAL_CHECK_TIMEOUT_S = 0.5

PRE_SCOOP_WAIT_S = 0.5


class SensorFrameLike(Protocol):
    ball_raw: int


class BallPickupBase(Protocol):
    def set_reload_servo(self, angle_deg: float) -> None: ...
    def set_fan_percent(self, percent: float) -> None: ...
    def read_sensors(self, timeout_s: float = 2.0) -> Optional[SensorFrameLike]: ...


class BallPickupArm(Protocol):
    def set_angle(self, angle: float, move_time_ms: int = 0) -> None: ...


def run_ball_pickup(
    base: BallPickupBase,
    arm: BallPickupArm,
    *,
    sleep: Callable[[float], None] = time.sleep,
    now: Callable[[], float] = time.monotonic,
) -> bool:
    """ボール回収シーケンスを実行する。戻り値: ボールを保持できたか。"""
    # 1. アームサーボ・リロードサーボを 0 度へ
    arm.set_angle(ARM_HOME_DEG)
    base.set_reload_servo(RELOAD_HOME_DEG)

    # 2. 0.5 秒待つ
    sleep(PRE_SCOOP_WAIT_S)

    # 3. リロードサーボを 140 度へ
    base.set_reload_servo(RELOAD_SCOOP_DEG)

    # 4. アームサーボを 1000ms で 103 度へ
    move_start = now()
    arm.set_angle(ARM_GRAB_DEG, move_time_ms=ARM_MOVE_TIME_MS)

    # 5. 到達(1000ms経過)したらファン Duty 50%
    _sleep_until_elapsed(sleep, now, move_start, ARM_MOVE_TIME_MS / 1000.0)
    base.set_fan_percent(FAN_ON_PERCENT)

    # 6. ball_raw > 100 が3回連続 → アーム 0度へ。2秒以内に検出できなければ失敗終了
    if not _wait_ball_detected(base, sleep=sleep, now=now):
        base.set_reload_servo(RELOAD_HOME_DEG)
        return False

    move_start = now()
    arm.set_angle(ARM_HOME_DEG, move_time_ms=ARM_MOVE_TIME_MS)

    # 7. アームが 0 度に到達(1000ms経過)したらファン Duty 0%
    _sleep_until_elapsed(sleep, now, move_start, ARM_MOVE_TIME_MS / 1000.0)
    base.set_fan_percent(FAN_OFF_PERCENT)

    # 8/9. 最終確認: ball_raw が 100 未満なら保持失敗、以上なら保持成功
    frame = base.read_sensors(timeout_s=BALL_FINAL_CHECK_TIMEOUT_S)
    ball_raw = frame.ball_raw if frame is not None else 0
    if ball_raw < BALL_THRESHOLD:
        base.set_reload_servo(RELOAD_HOME_DEG)
        return False

    return True


def _sleep_until_elapsed(
    sleep: Callable[[float], None], now: Callable[[], float], start: float, duration_s: float
) -> None:
    remaining = duration_s - (now() - start)
    if remaining > 0:
        sleep(remaining)


def _wait_ball_detected(
    base: BallPickupBase, *, sleep: Callable[[float], None], now: Callable[[], float]
) -> bool:
    deadline = now() + BALL_WAIT_TIMEOUT_S
    streak = 0
    while now() < deadline:
        frame = base.read_sensors(timeout_s=BALL_POLL_TIMEOUT_S)
        if frame is not None and frame.ball_raw > BALL_THRESHOLD:
            streak += 1
            if streak >= BALL_CONSECUTIVE:
                return True
        else:
            streak = 0
    return False
