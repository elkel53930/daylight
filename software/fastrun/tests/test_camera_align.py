"""camera_align / vision_wall のユニットテスト(合成画像、ハード非依存)。"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from camera_align import (
    CAMERA_DIST_GAIN_PX_PER_MM,
    CAMERA_DIST_INTERCEPT_PX,
    CAMERA_YAW_BIAS_DEG,
    CAMERA_YAW_SLOPE_GAIN,
    CAMERA_YAW_SLOPE_INTERCEPT_DEG,
    PoseEstimate,
    estimate_pose,
    is_confident,
)
from vision_wall import detect_red_band_top_edge

RED = (230, 80, 80)


def make_band_image(w: int, h: int, slope: float, intercept: float,
                    thickness: int = 90) -> np.ndarray:
    """上端が row = slope*col + intercept の赤帯を持つ合成RGB画像。"""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    for col in range(w):
        top = int(round(slope * col + intercept))
        top = max(0, min(h - 1, top))
        bot = min(h, top + thickness)
        img[top:bot, col] = RED
    return img


class TestVisionWall(unittest.TestCase):
    def test_recovers_horizontal_edge(self):
        img = make_band_image(600, 800, slope=0.0, intercept=400.0)
        edge = detect_red_band_top_edge(img)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, 0.0, places=3)
        self.assertAlmostEqual(edge.intercept, 400.0, delta=1.0)

    def test_recovers_tilted_edge(self):
        img = make_band_image(600, 800, slope=0.15, intercept=200.0)
        edge = detect_red_band_top_edge(img)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, 0.15, delta=0.01)

    def test_no_red_returns_none(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        self.assertIsNone(detect_red_band_top_edge(img))


class TestEstimatePose(unittest.TestCase):
    def test_horizontal_edge_yaw_matches_formula(self):
        # 水平エッジ(slope=0)。クロップは水平方向のみなので slope 不変。
        img = make_band_image(800, 1000, slope=0.0, intercept=691.55)
        est = estimate_pose(img)
        self.assertIsNotNone(est)
        expected_yaw = (0.0 - CAMERA_YAW_SLOPE_INTERCEPT_DEG) / CAMERA_YAW_SLOPE_GAIN + CAMERA_YAW_BIAS_DEG
        self.assertAlmostEqual(est.yaw_deg, expected_yaw, delta=0.2)
        # intercept≈691.55 なので dist_offset≈0
        self.assertAlmostEqual(est.dist_offset_mm, 0.0, delta=1.0)

    def test_dist_offset_sign(self):
        # 赤帯が下側(row大)に写る = より手前(=基準より前進側にいる) → dist>0
        img_near = make_band_image(800, 1000, slope=0.0, intercept=750.0)
        est = estimate_pose(img_near)
        self.assertIsNotNone(est)
        self.assertGreater(est.dist_offset_mm, 0.0)
        expected = (750.0 - CAMERA_DIST_INTERCEPT_PX) / CAMERA_DIST_GAIN_PX_PER_MM
        self.assertAlmostEqual(est.dist_offset_mm, expected, delta=1.0)

    def test_tilt_changes_yaw_monotonically(self):
        # 傾きが増えるとヨー角推定も単調に増える(符号の一貫性)。
        est0 = estimate_pose(make_band_image(800, 1000, 0.00, 500.0))
        est1 = estimate_pose(make_band_image(800, 1000, 0.10, 500.0))
        self.assertIsNotNone(est0)
        self.assertIsNotNone(est1)
        self.assertGreater(est1.yaw_deg, est0.yaw_deg)


class TestConfidenceGate(unittest.TestCase):
    def _mk(self, yaw=0.0, dist=0.0, res=0.5, n=100):
        return PoseEstimate(yaw_deg=yaw, dist_offset_mm=dist, residual_px=res, inlier_count=n)

    def test_accepts_reasonable(self):
        self.assertTrue(is_confident(self._mk(yaw=5.0, dist=10.0, res=1.0)))

    def test_rejects_large_yaw(self):
        self.assertFalse(is_confident(self._mk(yaw=20.0)))

    def test_rejects_large_dist(self):
        self.assertFalse(is_confident(self._mk(dist=60.0)))

    def test_rejects_large_residual(self):
        self.assertFalse(is_confident(self._mk(res=5.0)))


if __name__ == "__main__":
    unittest.main()
