import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from camera_correction import (
    CAMERA_CORRECTION_MAX_DIST_MM,
    CAMERA_CORRECTION_MAX_RESIDUAL_PX,
    CAMERA_CORRECTION_MAX_YAW_DEG,
    CAMERA_DIST_GAIN_PX_PER_MM,
    CAMERA_DIST_INTERCEPT_PX,
    CAMERA_YAW_BIAS_DEG,
    CAMERA_YAW_SLOPE_GAIN,
    CAMERA_YAW_SLOPE_INTERCEPT_DEG,
    PoseEstimate,
    estimate_pose,
    is_confident,
)

WHITE = (200, 200, 200)
RED = (230, 70, 90)


def make_full_image(h: int, w: int, slope: float, intercept: float, band_height: int = 30) -> np.ndarray:
    """フルサイズ画像(クロップ前)に、指定した傾き・切片(フル画像座標系)の
    赤帯を1本描く。estimate_pose()は内部で中央50%をクロップするため、
    クロップ座標系での傾き・切片への変換はテスト側で計算する。
    """
    img = np.full((h, w, 3), WHITE, dtype=np.uint8)
    for col in range(w):
        top = int(round(slope * col + intercept))
        bottom = min(h, top + band_height)
        if 0 <= top < h:
            img[top:bottom, col] = RED
    return img


class TestEstimatePose(unittest.TestCase):
    def test_known_slope_and_position_recovers_yaw_and_distance(self):
        h, w = 1000, 400
        true_slope = 0.02
        true_intercept = 690.0  # フル画像座標系(col=0での行)
        img = make_full_image(h, w, true_slope, true_intercept)

        estimate = estimate_pose(img)
        self.assertIsNotNone(estimate)

        # クロップ座標系(w//4 〜 3*w//4)での切片・中央位置を手計算し、
        # 実装と同じ式で期待値を出す(vision.py自体の精度はtest_vision.pyで
        # 別途検証済みなので、ここではcamera_correction.py側の配線
        # (クロップ範囲・較正式の適用)を検証する)。
        crop_x0 = w // 4
        cropped_w = 3 * w // 4 - crop_x0
        cropped_intercept = true_slope * crop_x0 + true_intercept

        expected_slope_deg = math.degrees(math.atan(true_slope))
        expected_yaw_deg = (
            expected_slope_deg - CAMERA_YAW_SLOPE_INTERCEPT_DEG
        ) / CAMERA_YAW_SLOPE_GAIN + CAMERA_YAW_BIAS_DEG
        self.assertAlmostEqual(estimate.yaw_deg, expected_yaw_deg, delta=0.1)

        expected_row_at_center = true_slope * (cropped_w / 2) + cropped_intercept
        expected_dist_mm = (
            expected_row_at_center - CAMERA_DIST_INTERCEPT_PX
        ) / CAMERA_DIST_GAIN_PX_PER_MM
        self.assertAlmostEqual(estimate.dist_offset_mm, expected_dist_mm, places=0)

        self.assertLess(estimate.residual_px, 1.0)

    def test_no_red_returns_none(self):
        img = np.full((200, 200, 3), WHITE, dtype=np.uint8)
        self.assertIsNone(estimate_pose(img))


class TestIsConfident(unittest.TestCase):
    def _make(self, *, residual_px=0.5, yaw_deg=0.0, dist_offset_mm=0.0):
        return PoseEstimate(
            yaw_deg=yaw_deg,
            dist_offset_mm=dist_offset_mm,
            residual_px=residual_px,
            inlier_count=200,
        )

    def test_clean_estimate_is_confident(self):
        self.assertTrue(is_confident(self._make()))

    def test_high_residual_is_rejected(self):
        e = self._make(residual_px=CAMERA_CORRECTION_MAX_RESIDUAL_PX + 0.1)
        self.assertFalse(is_confident(e))

    def test_large_yaw_is_rejected(self):
        e = self._make(yaw_deg=CAMERA_CORRECTION_MAX_YAW_DEG + 1.0)
        self.assertFalse(is_confident(e))
        e_neg = self._make(yaw_deg=-(CAMERA_CORRECTION_MAX_YAW_DEG + 1.0))
        self.assertFalse(is_confident(e_neg))

    def test_large_distance_offset_is_rejected(self):
        e = self._make(dist_offset_mm=CAMERA_CORRECTION_MAX_DIST_MM + 1.0)
        self.assertFalse(is_confident(e))
        e_neg = self._make(dist_offset_mm=-(CAMERA_CORRECTION_MAX_DIST_MM + 1.0))
        self.assertFalse(is_confident(e_neg))

    def test_boundary_values_are_confident(self):
        e = self._make(
            residual_px=CAMERA_CORRECTION_MAX_RESIDUAL_PX,
            yaw_deg=CAMERA_CORRECTION_MAX_YAW_DEG,
            dist_offset_mm=CAMERA_CORRECTION_MAX_DIST_MM,
        )
        self.assertTrue(is_confident(e))


if __name__ == "__main__":
    unittest.main()
