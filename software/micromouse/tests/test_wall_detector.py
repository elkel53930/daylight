import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from wall_detector import SensorFrame, WallDetector, parse_sen_line


def make_frame(lf=0, ls=0, rs=0, rf=0, vbatt=7.4) -> SensorFrame:
    return SensorFrame(
        gyro_radps=0.0,
        vbatt=vbatt,
        lf=lf,
        ls=ls,
        rs=rs,
        rf=rf,
        enc_r=0,
        enc_l=0,
        odo_dist_mm=0.0,
        odo_ang_rad=0.0,
    )


class TestParseSenLine(unittest.TestCase):
    def test_valid_line(self):
        line = "SEN,0.01,7.42,120,250,240,130,1000,2000,90.50,0.02"
        frame = parse_sen_line(line)
        self.assertIsNotNone(frame)
        self.assertAlmostEqual(frame.gyro_radps, 0.01)
        self.assertAlmostEqual(frame.vbatt, 7.42)
        self.assertEqual(frame.lf, 120)
        self.assertEqual(frame.ls, 250)
        self.assertEqual(frame.rs, 240)
        self.assertEqual(frame.rf, 130)
        self.assertEqual(frame.enc_r, 1000)
        self.assertEqual(frame.enc_l, 2000)
        self.assertAlmostEqual(frame.odo_dist_mm, 90.5)
        self.assertAlmostEqual(frame.odo_ang_rad, 0.02)

    def test_invalid_lines(self):
        self.assertIsNone(parse_sen_line("DONE"))
        self.assertIsNone(parse_sen_line("SEN,1,2,3"))  # フィールド不足
        self.assertIsNone(parse_sen_line("SEN,a,b,c,d,e,f,g,h,i,j"))  # 数値でない
        self.assertIsNone(parse_sen_line("#SEN,comment"))
        self.assertIsNone(parse_sen_line(""))


class TestWallDetector(unittest.TestCase):
    def setUp(self):
        self.detector = WallDetector(
            left_threshold=100, right_threshold=100, front_threshold=50
        )

    def test_all_walls(self):
        obs = self.detector.detect(make_frame(lf=60, ls=150, rs=150, rf=60))
        self.assertTrue(obs.left)
        self.assertTrue(obs.front)
        self.assertTrue(obs.right)

    def test_no_walls(self):
        obs = self.detector.detect(make_frame(lf=10, ls=20, rs=20, rf=10))
        self.assertFalse(obs.left)
        self.assertFalse(obs.front)
        self.assertFalse(obs.right)

    def test_threshold_boundary(self):
        # しきい値ちょうどは「壁あり」
        obs = self.detector.detect(make_frame(lf=50, ls=100, rs=99, rf=50))
        self.assertTrue(obs.left)
        self.assertTrue(obs.front)
        self.assertFalse(obs.right)

    def test_front_requires_both(self):
        # Daylight では lf == rf だが、判定は min(lf, rf) を使う
        obs = self.detector.detect(make_frame(lf=60, rf=10))
        self.assertFalse(obs.front)

    def test_sensor_sanity(self):
        self.assertTrue(self.detector.is_sensor_sane(make_frame(lf=100)))
        self.assertFalse(self.detector.is_sensor_sane(make_frame(lf=5000)))


if __name__ == "__main__":
    unittest.main()
