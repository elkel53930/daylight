"""recenter の位置成分(前後オフセット推定)ユニットテスト(合成画像、ハード非依存)。"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from recenter import (
    CAMERA_ROW_AT_90MM,
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
    # forward_offset_from_row は camera_model(距離認識モデル)へ委譲。較正点で確認。
    def test_center_row_is_zero_offset(self):
        # 90mm の較正 row=841.0 → offset≈0
        self.assertAlmostEqual(forward_offset_from_row(841.0), 0.0, delta=0.2)

    def test_row_below_center_is_positive(self):
        # 85mm の row=925.6(画面下=近い)= 中心より前 = +5mm 付近
        self.assertAlmostEqual(forward_offset_from_row(925.6), 5.0, delta=0.5)

    def test_row_above_center_is_negative(self):
        # 100mm の row=736.5(遠い)= −10mm 付近
        self.assertAlmostEqual(forward_offset_from_row(736.5), -10.0, delta=0.5)


class TestForwardOffsetFromImage(unittest.TestCase):
    # forward_offset_from_image は「下端エッジ」を使う(2026-08-04〜)。合成帯の下端
    # (top+thickness)が基準rowに来るよう top を置く。TH=帯の厚み。
    TH = 90

    # 検出は row を CALIB_HEIGHT(1296)へスケールするので、テスト画像も高さ1296で
    # 作れば scale=1 となり、下端が target_row(較正基準)に一致する。
    def _band_bottom_at(self, target_row, *, w=800, slope=0.0):
        from camera_align import CALIB_HEIGHT
        return make_band_image(w, CALIB_HEIGHT, slope=slope,
                               intercept=target_row - self.TH, thickness=self.TH)

    def test_center_image_zero_offset_confident(self):
        # 下端が 90mm の較正row=841.0 → offset≈0、較正範囲内なので confident
        est = forward_offset_from_image(self._band_bottom_at(841.0))
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.offset_mm, 0.0, delta=1.0)
        self.assertTrue(est.confident)

    def test_forward_offset_sign_and_magnitude(self):
        # 85mm の較正row=925.6(中心より前=壁に近い) → offset≈+5mm
        est = forward_offset_from_image(self._band_bottom_at(925.6))
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.offset_mm, 5.0, delta=1.5)
        self.assertGreater(est.offset_mm, 0.0)

    def test_no_red_returns_none(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        self.assertIsNone(forward_offset_from_image(img))

    def test_out_of_range_not_confident(self):
        # 較正範囲(row 607〜1141)外=近すぎ(row>1141)は confident=False(移動に使わせない)
        est = forward_offset_from_image(self._band_bottom_at(1210.0))
        self.assertIsNotNone(est)
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
