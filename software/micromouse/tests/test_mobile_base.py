"""MobileBase が新 mob プロトコル(JOGFWD/JOGTURN/MOT)を送ることを検証する。

実機・pyserial のポートは使わず、serial.Serial をフェイクに差し替えて
送信文字列だけを確認する。
"""

import sys
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent.parent))

import mobile_base
from mobile_base import MobileBase


class FakeSerial:
    instances = []

    def __init__(self, *args, **kwargs):
        self.writes = []
        self._responses = deque()
        FakeSerial.instances.append(self)

    def reset_input_buffer(self):
        self._responses.clear()

    def reset_output_buffer(self):
        pass

    def write(self, data):
        line = data.decode("ascii").strip()
        self.writes.append(line)
        if line.startswith(("JOG", "SANG", "RANG", "RDST", "GCAL")):
            self._responses.append(b"DONE\n")
        elif line in ("PCLEAR",) or line.startswith("PADD,"):
            # PRUN系のDONE応答
            self._responses.append(b"DONE\n")
        elif line == "PRUN":
            # PRUN完了時のDONE応答(2026-11-03追加: mob側で全セグメント完了時にDONE送信)
            self._responses.append(b"DONE\n")
        elif line == "SEN":
            # デフォルトのSEN応答（テストで上書き可能）
            if not self._responses:
                self._responses.append(
                    b"SEN,0.00,7.40,100,200,200,100,0,0,0.00,0.00,0,0\n"
                )
        return len(data)

    def readline(self):
        if self._responses:
            return self._responses.popleft()
        return b""

    def close(self):
        pass


class TestMobileBaseProtocol(unittest.TestCase):
    def setUp(self):
        FakeSerial.instances = []
        patcher = patch.object(mobile_base.serial, "Serial", FakeSerial)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.base = MobileBase("/dev/fake")
        self.fake = FakeSerial.instances[-1]

    def assertWrites(self, *expected):
        self.assertEqual(self.fake.writes, list(expected))

    def test_forward_uses_jogfwd(self):
        self.base.forward(300, 1000, 90)
        self.assertWrites("JOGFWD,90.0")

    def test_stop_at_releases_hold(self):
        self.base.stop_at(300, 1000, 90)
        self.assertWrites("JOGFWD,90.0", "MOT,0,0")

    def test_turn_uses_jogturn(self):
        import math

        self.base.turn(math.pi / 2)
        self.assertWrites("JOGTURN,1.570796")

    def test_jog_backward(self):
        self.base.jog_backward(-25)
        self.assertWrites("JOGBACK,25.0")

    def test_reset_commands_release_hold_first(self):
        self.base.reset_angle()
        self.assertWrites("MOT,0,0", "RANG")

        self.fake.writes.clear()
        self.base.reset_distance()
        self.assertWrites("MOT,0,0", "RDST")

    def test_correct_angle_releases_hold_first(self):
        self.base.correct_angle(0.0)
        self.assertWrites("MOT,0,0", "SANG,0.000000")

    def test_emergency_stop_is_mot_zero(self):
        self.base.emergency_stop()
        self.assertWrites("MOT,0,0", "MOT,0,0")

    def test_long_jog_timeout_scales_with_distance(self):
        short = self.base._jog_timeout(180)
        long = self.base._jog_timeout(2880)
        self.assertEqual(short, 10.0)
        self.assertGreater(long, short)

    def test_gyro_calibrate_releases_hold(self):
        with patch.object(mobile_base.time, "sleep", lambda *_: None):
            self.base.gyro_calibrate()
        self.assertWrites("MOT,0,0", "GCAL")

    def test_jog_forward(self):
        self.base.jog_forward(50)
        self.assertWrites("JOGFWD,50.0")

    def test_jog_turn(self):
        import math

        self.base.jog_turn(math.pi)
        self.assertWrites("JOGTURN,3.141593")

    def test_quick_stop(self):
        result = self.base.quick_stop()
        self.assertWrites("MOT,0,0")
        self.assertEqual(result, 0.0)

    def test_wall_led_on(self):
        with patch.object(mobile_base.time, "sleep", lambda *_: None):
            self.base.wall_led(True)
        self.assertWrites("WALL,1")

    def test_wall_led_off(self):
        with patch.object(mobile_base.time, "sleep", lambda *_: None):
            self.base.wall_led(False)
        self.assertWrites("WALL,0")

    def test_zero_distance_is_noop(self):
        self.base.forward(300, 1000, 0)
        self.assertWrites()

    def test_zero_angle_is_noop(self):
        self.base.turn(0)
        self.assertWrites()

    def test_read_sensors(self):
        # read_sensorsはSENコマンドを送信して応答を待つ
        # FakeSerialはSENコマンドに対して自動応答する
        frame = self.base.read_sensors()
        self.assertIsNotNone(frame)
        self.assertEqual(frame.vbatt, 7.40)
        self.assertEqual(frame.lf, 100)
        self.assertEqual(frame.ls, 200)
        self.assertEqual(frame.rs, 200)
        self.assertEqual(frame.rf, 100)

    def test_forward_with_wall_feedback(self):
        """壁センサフィードバック付き前進のテスト。
        
        事前条件:
        - FakeSerial が PCLEAR/PADD/PRUN の DONE を返す
        
        事後条件:
        - PCLEAR が送信される
        - PADD,STRAIGHT が送信される
        - PRUN が送信される
        - DONE を受信して完了検出
        - MOT,0,0 を送信して PATH_FOLLOW を抜ける
        """
        self.base.forward_with_wall_feedback(100.0)
        
        # 送信されたコマンドを確認
        self.assertEqual(self.fake.writes[0], "PCLEAR")
        self.assertEqual(self.fake.writes[1], "PADD,STRAIGHT,100.0,0.0,300.0,0.0")
        self.assertEqual(self.fake.writes[2], "PRUN")
        # 最後に MOT,0,0 で停止
        self.assertEqual(self.fake.writes[-1], "MOT,0,0")


if __name__ == "__main__":
    unittest.main()
