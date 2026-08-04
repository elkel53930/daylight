"""camera_model(距離認識モデル)のユニットテスト(ハード非依存)。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from camera_model import (
    CELL_CENTER_MM,
    distance_from_row,
    estimate,
    forward_offset_mm,
    gain_at,
    is_in_calib_range,
    straight_at,
)


class TestDistanceFromRow(unittest.TestCase):
    def test_recovers_calibration_points(self):
        self.assertAlmostEqual(distance_from_row(841.0), 90.0, delta=0.3)
        self.assertAlmostEqual(distance_from_row(1141.4), 75.0, delta=0.3)
        self.assertAlmostEqual(distance_from_row(607.2), 115.0, delta=0.3)

    def test_monotonic_higher_row_is_closer(self):
        # row 大 = 壁が近い = 距離小
        self.assertLess(distance_from_row(900.0), distance_from_row(800.0))

    def test_interpolates_between(self):
        # 85mm(925.6)と90mm(841.0)の間の row → 85〜90mm
        d = distance_from_row(880.0)
        self.assertTrue(85.0 < d < 90.0)

    def test_out_of_range_clamps(self):
        self.assertEqual(distance_from_row(2000.0), 75.0)   # 近すぎ側クランプ
        self.assertEqual(distance_from_row(100.0), 115.0)   # 遠すぎ側クランプ


class TestGainStraight(unittest.TestCase):
    def test_recovers_points(self):
        self.assertAlmostEqual(gain_at(75.0), 0.809, delta=0.001)
        self.assertAlmostEqual(gain_at(90.0), 0.608, delta=0.001)
        self.assertAlmostEqual(gain_at(115.0), 0.431, delta=0.001)
        self.assertAlmostEqual(straight_at(90.0), 0.787, delta=0.001)

    def test_gain_decreases_with_distance(self):
        self.assertGreater(gain_at(80.0), gain_at(100.0))


class TestEstimate(unittest.TestCase):
    def test_squared_at_90(self):
        d, y = estimate(841.0, 0.787)  # 90mm・正対
        self.assertAlmostEqual(d, 90.0, delta=0.5)
        self.assertAlmostEqual(y, 0.0, delta=0.2)

    def test_small_yaw_at_90(self):
        # 90mm・yaw+2°(実測 row≈836, slope≈1.970)
        d, y = estimate(836.0, 1.970)
        self.assertAlmostEqual(d, 90.0, delta=2.0)
        self.assertAlmostEqual(y, 2.0, delta=0.5)

    def test_negative_yaw(self):
        # 90mm・yaw−2°(実測 row≈837.9, slope≈−0.629)
        d, y = estimate(837.9, -0.629)
        self.assertLess(y, 0.0)
        self.assertAlmostEqual(y, -2.0, delta=0.5)


class TestForwardOffset(unittest.TestCase):
    def test_center_zero(self):
        self.assertAlmostEqual(forward_offset_mm(841.0), 0.0, delta=0.3)

    def test_closer_is_positive(self):
        # 85mm(row 925.6)= 中心より前 → +5mm 付近
        self.assertGreater(forward_offset_mm(925.6), 0.0)
        self.assertAlmostEqual(forward_offset_mm(925.6), 5.0, delta=0.5)


class TestRange(unittest.TestCase):
    def test_range(self):
        self.assertTrue(is_in_calib_range(90.0))
        self.assertFalse(is_in_calib_range(70.0))
        self.assertFalse(is_in_calib_range(120.0))


if __name__ == "__main__":
    unittest.main()
