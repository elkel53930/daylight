import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).parent.parent))

from ball_pickup import (
    ARM_GRAB_DEG,
    ARM_HOME_DEG,
    RELOAD_HOME_DEG,
    RELOAD_SCOOP_DEG,
    run_ball_pickup,
)


@dataclass(frozen=True)
class FakeFrame:
    ball_raw: int


class FakeClock:
    """sleep() で時刻を進める決定論的な偽時計(テストを高速化する)。"""

    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            self.t += seconds


class FakeArm:
    def __init__(self):
        self.calls: List[tuple] = []

    def set_angle(self, angle: float, move_time_ms: int = 0) -> None:
        self.calls.append((angle, move_time_ms))


# read_sensors() が「応答があった」場合に消費する時間(実際のシリアル
# ラウンドトリップを模した小さな固定値)。
FAST_RESPONSE_S = 0.01


class FakeBase:
    """set_reload_servo/set_fan_percent の呼び出し記録 + ball_raw の応答列を返す。

    read_sensors() は実機同様「応答が無ければ timeout_s 分だけブロックする」
    ことを FakeClock 上でも再現する(そうしないと _wait_ball_detected の
    ポーリングループが偽時計上で無限ループになる)。
    """

    def __init__(self, clock: FakeClock, ball_raw_sequence: Optional[List[int]] = None):
        self.clock = clock
        self.calls: List[tuple] = []
        self._ball_raw_sequence = list(ball_raw_sequence) if ball_raw_sequence else []

    def set_reload_servo(self, angle_deg: float) -> None:
        self.calls.append(("reload", angle_deg))

    def set_fan_percent(self, percent: float) -> None:
        self.calls.append(("fan", percent))

    def read_sensors(self, timeout_s: float = 2.0):
        self.calls.append(("read_sensors", timeout_s))
        if self._ball_raw_sequence:
            value = self._ball_raw_sequence.pop(0)
            self.clock.t += FAST_RESPONSE_S
            return FakeFrame(ball_raw=value)
        self.clock.t += timeout_s  # 応答なし: timeout_s いっぱいブロックした扱い
        return None


def names(calls):
    return [c[0] for c in calls]


class TestBallPickupSuccess(unittest.TestCase):
    def test_full_sequence_on_success(self):
        clock = FakeClock()
        # 3連続で閾値超え→回収→最終確認も閾値超え(保持成功)
        base = FakeBase(clock, ball_raw_sequence=[150, 150, 150, 200])
        arm = FakeArm()

        result = run_ball_pickup(base, arm, sleep=clock.sleep, now=clock.now)

        self.assertTrue(result)
        # アーム: 0度(即時)→103度(1000ms)→0度(1000ms)
        self.assertEqual(arm.calls[0], (ARM_HOME_DEG, 0))
        self.assertEqual(arm.calls[1], (ARM_GRAB_DEG, 1000))
        self.assertEqual(arm.calls[2], (ARM_HOME_DEG, 1000))
        # リロード: 0度→140度(保持成功時は0度に戻さない)
        reload_calls = [c for c in base.calls if c[0] == "reload"]
        self.assertEqual(reload_calls, [("reload", RELOAD_HOME_DEG), ("reload", RELOAD_SCOOP_DEG)])
        # ファン: 50%→0%
        fan_calls = [c for c in base.calls if c[0] == "fan"]
        self.assertEqual(fan_calls, [("fan", 50.0), ("fan", 0.0)])

    def test_timing_waits_are_respected(self):
        clock = FakeClock()
        base = FakeBase(clock, ball_raw_sequence=[150, 150, 150, 200])
        arm = FakeArm()

        run_ball_pickup(base, arm, sleep=clock.sleep, now=clock.now)

        # 0.5秒待機 + アーム移動2回(各1000ms)は最低限経過している
        self.assertGreaterEqual(clock.t, 2.5)


class TestBallPickupTimeout(unittest.TestCase):
    def test_no_detection_within_timeout_resets_reload_and_fails(self):
        clock = FakeClock()
        base = FakeBase(clock, ball_raw_sequence=[])  # 常に None(検出なし)
        arm = FakeArm()

        result = run_ball_pickup(base, arm, sleep=clock.sleep, now=clock.now)

        self.assertFalse(result)
        # アームは103度まで動かし、タイムアウト時も0度へ復帰させる
        self.assertEqual(len(arm.calls), 3)
        self.assertEqual(arm.calls[1], (ARM_GRAB_DEG, 1000))
        self.assertEqual(arm.calls[2], (ARM_HOME_DEG, 1000))
        # リロードは 0度→140度→(タイムアウトで)0度
        reload_calls = [c[1] for c in base.calls if c[0] == "reload"]
        self.assertEqual(reload_calls, [RELOAD_HOME_DEG, RELOAD_SCOOP_DEG, RELOAD_HOME_DEG])
        # ファンは起動(50%)後、タイムアウト時もOFFに戻す
        fan_calls = [c[1] for c in base.calls if c[0] == "fan"]
        self.assertEqual(fan_calls, [50.0, 0.0])

    def test_non_consecutive_detections_do_not_count(self):
        clock = FakeClock()
        # 検出→非検出→検出→非検出... と連続しないので3連続に届かないまま
        # タイムアウトする
        base = FakeBase(clock, ball_raw_sequence=[150, 0, 150, 0, 150, 0, 150, 0])
        arm = FakeArm()

        result = run_ball_pickup(base, arm, sleep=clock.sleep, now=clock.now)

        self.assertFalse(result)


class TestBallPickupFinalCheckFails(unittest.TestCase):
    def test_ball_lost_before_final_check_resets_reload(self):
        clock = FakeClock()
        # 3連続検出はできるが、最終確認時点でball_rawが閾値未満に落ちている
        base = FakeBase(clock, ball_raw_sequence=[150, 150, 150, 50])
        arm = FakeArm()

        result = run_ball_pickup(base, arm, sleep=clock.sleep, now=clock.now)

        self.assertFalse(result)
        # アームは0度復帰(手順7)まで実行される
        self.assertEqual(len(arm.calls), 3)
        # リロードは最終的に0度へリセットされる
        reload_calls = [c[1] for c in base.calls if c[0] == "reload"]
        self.assertEqual(reload_calls[-1], RELOAD_HOME_DEG)
        # ファンは50%→0%まで実行される(最終確認はその後)
        fan_calls = [c[1] for c in base.calls if c[0] == "fan"]
        self.assertEqual(fan_calls, [50.0, 0.0])

    def test_final_check_returns_none_treated_as_no_ball(self):
        clock = FakeClock()
        base = FakeBase(clock, ball_raw_sequence=[150, 150, 150])  # 最終確認はNone(応答なし)
        arm = FakeArm()

        result = run_ball_pickup(base, arm, sleep=clock.sleep, now=clock.now)

        self.assertFalse(result)


if __name__ == "__main__":
    unittest.main()
