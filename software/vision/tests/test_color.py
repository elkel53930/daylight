import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from color import bgr_to_hsv, in_range, make_mask, mask_ratio
from vision_types import ColorRange


def solid_bgr(h, w, bgr):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = bgr
    return img


class TestBgrToHsv(unittest.TestCase):
    def test_pure_red(self):
        # BGR(0,0,255) は HSV で H≈0, S=255, V=255
        hsv = bgr_to_hsv(solid_bgr(4, 4, (0, 0, 255)))
        self.assertEqual(hsv[0, 0, 0], 0)
        self.assertEqual(hsv[0, 0, 1], 255)
        self.assertEqual(hsv[0, 0, 2], 255)

    def test_pure_green(self):
        # BGR(0,255,0) は H=120度 → OpenCV では 60
        hsv = bgr_to_hsv(solid_bgr(2, 2, (0, 255, 0)))
        self.assertEqual(hsv[0, 0, 0], 60)

    def test_pure_blue(self):
        # BGR(255,0,0) は H=240度 → OpenCV では 120
        hsv = bgr_to_hsv(solid_bgr(2, 2, (255, 0, 0)))
        self.assertEqual(hsv[0, 0, 0], 120)

    def test_yellow(self):
        # BGR(0,255,255) は H=60度 → OpenCV では 30
        hsv = bgr_to_hsv(solid_bgr(2, 2, (0, 255, 255)))
        self.assertEqual(hsv[0, 0, 0], 30)

    def test_gray_has_zero_saturation(self):
        hsv = bgr_to_hsv(solid_bgr(2, 2, (128, 128, 128)))
        self.assertEqual(hsv[0, 0, 1], 0)
        self.assertEqual(hsv[0, 0, 2], 128)

    def test_rejects_non_bgr(self):
        with self.assertRaises(ValueError):
            bgr_to_hsv(np.zeros((4, 4), dtype=np.uint8))


class TestMask(unittest.TestCase):
    def test_in_range_yellow(self):
        hsv = bgr_to_hsv(solid_bgr(3, 3, (0, 255, 255)))
        mask = in_range(hsv, ColorRange((20, 80, 80), (40, 255, 255)))
        self.assertTrue(mask.all())

    def test_make_mask_multiple_ranges_or(self):
        # 左半分を赤(H≈0)、右半分を別の赤帯(H≈179)にして 2 範囲 OR で両方拾う
        img = np.zeros((4, 4, 3), dtype=np.uint8)
        img[:, :2] = (0, 0, 255)      # H≈0
        img[:, 2:] = (10, 0, 255)     # わずかに青を混ぜて H を上げる
        ranges = [ColorRange((0, 80, 60), (10, 255, 255)),
                  ColorRange((160, 80, 60), (179, 255, 255))]
        mask = make_mask(img, ranges)
        self.assertTrue(mask[:, :2].all())

    def test_mask_ratio(self):
        mask = np.zeros((10, 10), dtype=bool)
        mask[:5, :] = True
        self.assertAlmostEqual(mask_ratio(mask), 0.5)


class TestResolutionIndependence(unittest.TestCase):
    def test_same_ratio_across_sizes(self):
        # 中央 1/4 を黄色にした画像は解像度に依らず割合が一定
        cr = ColorRange((20, 80, 80), (40, 255, 255))
        ratios = []
        for (h, w) in [(240, 320), (480, 640), (1080, 1920)]:
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[h // 4:3 * h // 4, w // 4:3 * w // 4] = (0, 255, 255)
            ratios.append(mask_ratio(make_mask(img, cr)))
        for r in ratios:
            self.assertAlmostEqual(r, 0.25, places=2)


if __name__ == "__main__":
    unittest.main()
