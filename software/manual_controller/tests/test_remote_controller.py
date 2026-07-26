import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent.parent))

import remote_protocol as proto
from remote_controller import RemoteController
from errors import AbortRequested


class FakeBase:
    """RemoteBase を満たすテスト用フェイク。呼び出しを記録する。"""

    def __init__(self):
        self.calls = []
        self.raise_on = None  # (method_name, exception) を設定するとそのメソッドで例外を送出

    def _record(self, name, *args):
        if self.raise_on is not None and self.raise_on[0] == name:
            raise self.raise_on[1]
        self.calls.append((name,) + args)

    def stop_at(self, speed_mmps, accel_mmps2, distance_mm):
        self._record("stop_at", speed_mmps, accel_mmps2, distance_mm)

    def turn(self, angle_rad):
        self._record("turn", angle_rad)

    def latch_forward(self):
        self._record("latch_forward")

    def latch_backward(self):
        self._record("latch_backward")

    def latch_turn_left(self):
        self._record("latch_turn_left")

    def latch_turn_right(self):
        self._record("latch_turn_right")

    def latch_stop(self):
        self._record("latch_stop")

    def emergency_stop(self):
        self._record("emergency_stop")

    def set_reload_servo(self, angle_deg):
        self._record("set_reload_servo", angle_deg)

    def set_fan_percent(self, percent):
        self._record("set_fan_percent", percent)

    def read_sensors(self, timeout_s=2.0):
        self._record("read_sensors", timeout_s)
        return None


class FakeArm:
    def __init__(self):
        self.calls = []

    def set_angle(self, angle, move_time_ms=0):
        self.calls.append((angle, move_time_ms))


def make_controller():
    base = FakeBase()
    ctrl = RemoteController(base, cell_speed_mmps=300.0, cell_accel_mmps2=1000.0, cell_size_mm=180.0)
    return ctrl, base


class TestDpadUpRepeatForward(unittest.TestCase):
    def test_held_repeats_cell_forward_each_step(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.step()
        ctrl.step()
        self.assertEqual(
            base.calls, [("stop_at", 300.0, 1000.0, 180.0)] * 3
        )

    def test_release_stops_further_cells(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()  # 1区間実行
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_UP)
        ctrl.step()  # もう動かない
        self.assertEqual(base.calls, [("stop_at", 300.0, 1000.0, 180.0)])

    def test_idle_step_calls_nothing(self):
        ctrl, base = make_controller()
        ctrl.step()
        self.assertEqual(base.calls, [])


class TestTurns(unittest.TestCase):
    def test_dpad_right_turns_minus_90(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("turn", -math.pi / 2)])

    def test_dpad_left_turns_plus_90(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_LEFT, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("turn", math.pi / 2)])

    def test_dpad_down_turns_180(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_DOWN, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("turn", math.pi)])

    def test_release_does_nothing(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_UP)
        ctrl.step()
        self.assertEqual(base.calls, [])

    def test_turn_fires_once_not_repeated(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.step()
        ctrl.step()
        self.assertEqual(base.calls, [("turn", -math.pi / 2)])

    def test_turn_does_not_block_subsequent_dpad_up(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()  # turn が優先
        ctrl.step()  # turn 消化済みなので dpad_up の前進
        self.assertEqual(
            base.calls, [("turn", -math.pi / 2), ("stop_at", 300.0, 1000.0, 180.0)]
        )

    def test_multiple_presses_before_any_step_are_all_queued_in_order(self):
        # 前の動作の完了待ち(stop_at/turnのブロッキング)中に複数回押しても
        # 単一変数では最後の1回しか残らず取りこぼしていた。キュー化により
        # 押した順に全て消化されることを保証する。
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.on_event(proto.DPAD_LEFT, proto.ACTION_DOWN)
        ctrl.on_event(proto.DPAD_DOWN, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.step()
        ctrl.step()
        self.assertEqual(
            base.calls,
            [("turn", -math.pi / 2), ("turn", math.pi / 2), ("turn", math.pi)],
        )

    def test_queue_empties_and_further_steps_do_nothing(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.step()
        base.calls.clear()
        ctrl.step()
        ctrl.step()
        self.assertEqual(base.calls, [])


class TestJogButtons(unittest.TestCase):
    def test_triangle_forward_start_stop(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_UP)
        ctrl.step()
        self.assertEqual(base.calls, [("latch_forward",), ("latch_stop",)])

    def test_circle_is_turn_right(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.CIRCLE, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("latch_turn_right",)])

    def test_cross_is_backward(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.CROSS, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("latch_backward",)])

    def test_square_is_turn_left(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.SQUARE, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("latch_turn_left",)])

    def test_held_does_not_resend_every_step(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.step()
        ctrl.step()
        self.assertEqual(base.calls, [("latch_forward",)])

    def test_switching_button_without_release_switches_latch(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.on_event(proto.CIRCLE, proto.ACTION_DOWN)  # △を離さずに○も押した想定
        ctrl.step()
        self.assertEqual(base.calls, [("latch_forward",), ("latch_turn_right",)])

    def test_release_of_stale_button_does_not_stop_new_one(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.on_event(proto.CIRCLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_UP)  # 後から△を離す
        ctrl.step()
        # △の解放は無視され、○のLATCHは継続(停止されない)
        self.assertEqual(base.calls, [("latch_forward",), ("latch_turn_right",)])

    def test_jog_takes_priority_over_pending_turn(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()  # jog優先
        ctrl.step()  # turn はまだ残っている
        self.assertEqual(
            base.calls, [("latch_forward",), ("turn", -math.pi / 2)]
        )


class TestDisconnect(unittest.TestCase):
    def test_handle_disconnect_emergency_stops_and_clears_state(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)

        ctrl.handle_disconnect()
        self.assertEqual(base.calls, [("emergency_stop",)])

        base.calls.clear()
        ctrl.step()
        ctrl.step()
        ctrl.step()
        self.assertEqual(base.calls, [])

    def test_reconnect_after_disconnect_resends_jog_start(self):
        # jog_sent もリセットされるので、再接続後に同じボタンイベントが
        # 来れば latch_forward が再送される(サーボ側は latch_stop 済みのため)。
        ctrl, base = make_controller()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.handle_disconnect()
        base.calls.clear()

        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("latch_forward",)])


class TestAbortDuringMotionDoesNotCrash(unittest.TestCase):
    def test_stop_at_raises_abort_is_swallowed(self):
        ctrl, base = make_controller()
        base.raise_on = ("stop_at", AbortRequested("link lost"))
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()  # 例外を吸収して落ちないこと
        self.assertEqual(base.calls, [])  # raise_on の記録はしない設計(_record内でraise)


class TestR1ReloadRelease(unittest.TestCase):
    def test_r1_sets_reload_servo_to_180(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.R1, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(base.calls, [("set_reload_servo", 180.0)])

    def test_r1_release_does_nothing(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.R1, proto.ACTION_UP)
        ctrl.step()
        self.assertEqual(base.calls, [])

    def test_r1_fires_once_not_repeated(self):
        ctrl, base = make_controller()
        ctrl.on_event(proto.R1, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.step()
        self.assertEqual(base.calls, [("set_reload_servo", 180.0)])


class TestL1BallPickup(unittest.TestCase):
    # run_ball_pickup() の中身(手順・タイミング・分岐)は test_ball_pickup.py
    # で高速な偽時計を使って検証済み。ここでは RemoteController が
    # 正しく配線されている(L1でキューに積まれ、step()で呼ばれ、戻り値が
    # ball_held に反映される)ことだけを、実際の待機を伴わずに確認する。

    def test_l1_without_arm_skips_gracefully(self):
        ctrl, base = make_controller()  # arm 未指定(None)
        ctrl.on_event(proto.L1, proto.ACTION_DOWN)
        ctrl.step()  # 例外を出さずスキップすること
        self.assertFalse(ctrl.ball_held)
        self.assertEqual(base.calls, [])

    def test_l1_success_sets_ball_held_true(self):
        ctrl, base = make_controller()
        arm = FakeArm()
        ctrl.arm = arm
        with patch("remote_controller.run_ball_pickup", return_value=True) as mock_run:
            ctrl.on_event(proto.L1, proto.ACTION_DOWN)
            ctrl.step()
        mock_run.assert_called_once_with(base, arm)
        self.assertTrue(ctrl.ball_held)

    def test_l1_failure_sets_ball_held_false(self):
        ctrl, base = make_controller()
        ctrl.arm = FakeArm()
        ctrl.ball_held = True  # 前回成功していた状態から
        with patch("remote_controller.run_ball_pickup", return_value=False):
            ctrl.on_event(proto.L1, proto.ACTION_DOWN)
            ctrl.step()
        self.assertFalse(ctrl.ball_held)

    def test_l1_exception_does_not_crash_worker(self):
        ctrl, base = make_controller()
        ctrl.arm = FakeArm()
        with patch("remote_controller.run_ball_pickup", side_effect=RuntimeError("boom")):
            ctrl.on_event(proto.L1, proto.ACTION_DOWN)
            ctrl.step()  # 例外を吸収して落ちないこと
        self.assertFalse(ctrl.ball_held)

    def test_l1_release_does_nothing(self):
        ctrl, base = make_controller()
        ctrl.arm = FakeArm()
        with patch("remote_controller.run_ball_pickup") as mock_run:
            ctrl.on_event(proto.L1, proto.ACTION_UP)
            ctrl.step()
        mock_run.assert_not_called()


class TestCommandDoneCallback(unittest.TestCase):
    """1区間前進・90/180度旋回の完了時のみ on_command_done を呼ぶこと。

    JOG(押しっぱなし)・L1・R1では呼ばれない(コントローラ振動フィードバックは
    「1コマンド完了」の合図であり、押しっぱなし操作には不要なため)。
    """

    def _make(self):
        base = FakeBase()
        done_calls = []
        ctrl = RemoteController(
            base,
            cell_speed_mmps=300.0,
            cell_accel_mmps2=1000.0,
            cell_size_mm=180.0,
            on_command_done=lambda: done_calls.append(1),
        )
        return ctrl, base, done_calls

    def test_dpad_up_success_notifies(self):
        ctrl, base, done_calls = self._make()
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(len(done_calls), 1)

    def test_dpad_up_repeats_notify_each_cell(self):
        ctrl, base, done_calls = self._make()
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.step()
        ctrl.step()
        self.assertEqual(len(done_calls), 3)

    def test_turn_success_notifies(self):
        ctrl, base, done_calls = self._make()
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(len(done_calls), 1)

    def test_failed_stop_at_does_not_notify(self):
        ctrl, base, done_calls = self._make()
        base.raise_on = ("stop_at", AbortRequested("link lost"))
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(done_calls, [])

    def test_failed_turn_does_not_notify(self):
        ctrl, base, done_calls = self._make()
        base.raise_on = ("turn", AbortRequested("link lost"))
        ctrl.on_event(proto.DPAD_RIGHT, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(done_calls, [])

    def test_jog_does_not_notify(self):
        ctrl, base, done_calls = self._make()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_DOWN)
        ctrl.step()
        ctrl.on_event(proto.TRIANGLE, proto.ACTION_UP)
        ctrl.step()
        self.assertEqual(done_calls, [])

    def test_l1_does_not_notify(self):
        ctrl, base, done_calls = self._make()
        ctrl.arm = FakeArm()
        with patch("remote_controller.run_ball_pickup", return_value=True):
            ctrl.on_event(proto.L1, proto.ACTION_DOWN)
            ctrl.step()
        self.assertEqual(done_calls, [])

    def test_r1_does_not_notify(self):
        ctrl, base, done_calls = self._make()
        ctrl.on_event(proto.R1, proto.ACTION_DOWN)
        ctrl.step()
        self.assertEqual(done_calls, [])

    def test_no_callback_configured_does_not_crash(self):
        ctrl, base = make_controller()  # on_command_done未指定(None)
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()  # 例外を出さないこと

    def test_callback_exception_does_not_crash_step(self):
        base = FakeBase()

        def bad_callback():
            raise RuntimeError("boom")

        ctrl = RemoteController(base, on_command_done=bad_callback)
        ctrl.on_event(proto.DPAD_UP, proto.ACTION_DOWN)
        ctrl.step()  # コールバック内例外を吸収して落ちないこと


if __name__ == "__main__":
    unittest.main()
