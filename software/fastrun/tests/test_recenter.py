"""recenter の位置成分(前後オフセット推定)ユニットテスト(合成画像、ハード非依存)。"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from recenter import (
    CAMERA_ROW_AT_90MM,
    CAMERA_ROW_PX_PER_MM,
    FORWARD_OFFSET_MAX_MM,
    FORWARD_OFFSET_MAX_RES,
    ROW_NEAR_STOP,
    ForwardOffset,
    approach_step_mm,
    forward_offset_from_image,
    forward_offset_from_row,
)

RED = (230, 80, 80)


def make_band_image(w: int, h: int, slope: float, intercept: float,
                    thickness: int = 90) -> np.ndarray:
    """上端が row = slope*col + intercept の赤帯を持つ合成RGB画像(test_camera_alignと同型)。"""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    for col in range(w):
        top = int(round(slope * col + intercept))
        top = max(0, min(h - 1, top))
        bot = min(h, top + thickness)
        img[top:bot, col] = RED
    return img


class TestForwardOffsetFromRow(unittest.TestCase):
    def test_center_row_is_zero_offset(self):
        self.assertAlmostEqual(forward_offset_from_row(CAMERA_ROW_AT_90MM), 0.0, places=6)

    def test_row_below_center_is_positive(self):
        # row が大きい(画面下=壁が近い)= 中心より前 = 正
        off = forward_offset_from_row(CAMERA_ROW_AT_90MM + CAMERA_ROW_PX_PER_MM * 10.0)
        self.assertAlmostEqual(off, 10.0, places=3)

    def test_row_above_center_is_negative(self):
        off = forward_offset_from_row(CAMERA_ROW_AT_90MM - CAMERA_ROW_PX_PER_MM * 7.0)
        self.assertAlmostEqual(off, -7.0, places=3)


class TestForwardOffsetFromImage(unittest.TestCase):
    # forward_offset_from_image は「下端エッジ」を使う(2026-08-04〜)。合成帯の下端
    # (top+thickness)が基準rowに来るよう top を置く。TH=帯の厚み。
    TH = 90

    def _band_bottom_at(self, target_row, *, w=800, slope=0.0, margin=140):
        h = int(target_row + margin)
        return make_band_image(w, h, slope=slope, intercept=target_row - self.TH,
                               thickness=self.TH)

    def test_center_image_zero_offset_confident(self):
        # 下端が CAMERA_ROW_AT_90MM → offset≈0
        est = forward_offset_from_image(self._band_bottom_at(CAMERA_ROW_AT_90MM))
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.offset_mm, 0.0, delta=3.0)
        self.assertTrue(est.confident)

    def test_forward_offset_sign_and_magnitude(self):
        # 中心より 20mm 前(壁に近い) → 下端row = 基準 + 7.647*20
        row = CAMERA_ROW_AT_90MM + CAMERA_ROW_PX_PER_MM * 20.0
        est = forward_offset_from_image(self._band_bottom_at(row))
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.offset_mm, 20.0, delta=3.0)
        self.assertGreater(est.offset_mm, 0.0)

    def test_no_red_returns_none(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        self.assertIsNone(forward_offset_from_image(img))

    def test_out_of_range_offset_not_confident(self):
        # 範囲外(>45mm)の大きなオフセットは confident=False(移動に使わせない)
        row = CAMERA_ROW_AT_90MM + CAMERA_ROW_PX_PER_MM * (FORWARD_OFFSET_MAX_MM + 15.0)
        est = forward_offset_from_image(self._band_bottom_at(row))
        self.assertIsNotNone(est)
        self.assertGreater(abs(est.offset_mm), FORWARD_OFFSET_MAX_MM)
        self.assertFalse(est.confident)

    def test_crop_frac_row_invariant(self):
        # row_at_center は水平中心の行なのでクロップ幅にほぼ不変。
        img = self._band_bottom_at(600.0, slope=0.05)
        e30 = forward_offset_from_image(img, crop_frac=0.3)
        e50 = forward_offset_from_image(img, crop_frac=0.5)
        self.assertIsNotNone(e30)
        self.assertIsNotNone(e50)
        self.assertAlmostEqual(e30.offset_mm, e50.offset_mm, delta=2.0)


class TestApproachStep(unittest.TestCase):
    def test_far_uses_large_step(self):
        # 遠い(row小)ほど大きく刻む。
        self.assertEqual(approach_step_mm(200.0), 30.0)

    def test_near_uses_small_step(self):
        # 近い(row大=急変域)ほど小さく刻み、行き過ぎを防ぐ(下端較正: row>=710で6mm)。
        self.assertEqual(approach_step_mm(760.0), 6.0)

    def test_step_monotonic_nonincreasing_with_row(self):
        rows = [100, 340, 360, 480, 520, 590, 610, 700]
        steps = [approach_step_mm(r) for r in rows]
        for a, b in zip(steps, steps[1:]):
            self.assertGreaterEqual(a, b)

    def test_near_stop_row_maps_below_90mm_distance(self):
        # ROW_NEAR_STOP(接近停止)は90mm手前(row<row@90mm)で止まる=近づきすぎない。
        self.assertLess(ROW_NEAR_STOP, CAMERA_ROW_AT_90MM)


if __name__ == "__main__":
    unittest.main()
