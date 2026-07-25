import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from ball import (
    detect_yellow_ball,
    estimate_yellow_ball,
    largest_contour_points,
)
from vision_types import BallEstimationConfig, ColorRange

YELLOW = ColorRange((20, 80, 80), (40, 255, 255))
YELLOW_BGR = (0, 255, 255)


def blank(h, w):
    return np.zeros((h, w, 3), dtype=np.uint8)


def draw_disk(img, cx, cy, r, bgr):
    h, w = img.shape[:2]
    ys, xs = np.ogrid[:h, :w]
    mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
    img[mask] = bgr
    return img


class TestDetectYellowBall(unittest.TestCase):
    def test_present(self):
        img = draw_disk(blank(240, 320), 160, 120, 50, YELLOW_BGR)
        self.assertTrue(detect_yellow_ball(img, YELLOW, threshold=0.01))

    def test_absent(self):
        img = blank(240, 320)  # 真っ黒
        self.assertFalse(detect_yellow_ball(img, YELLOW, threshold=0.01))

    def test_small_noise_below_threshold(self):
        img = blank(240, 320)
        img[0:3, 0:3] = YELLOW_BGR  # ごく小さな黄色
        self.assertFalse(detect_yellow_ball(img, YELLOW, threshold=0.05))

    def test_resolution_independent_decision(self):
        # 同じ相対サイズの円なら解像度が変わっても判定は一致
        for (h, w) in [(240, 320), (480, 640), (1080, 1920)]:
            img = draw_disk(blank(h, w), w // 2, h // 2, min(h, w) // 6, YELLOW_BGR)
            self.assertTrue(detect_yellow_ball(img, YELLOW, threshold=0.01))


class TestLargestContourPoints(unittest.TestCase):
    def test_contour_is_ring_not_filled(self):
        img = draw_disk(blank(200, 200), 100, 100, 40, YELLOW_BGR)
        from color import make_mask
        mask = make_mask(img, YELLOW)
        pts = largest_contour_points(mask)
        # 輪郭点数は塗り潰し面積よりずっと少なく、円周長 2πr に近いオーダ
        self.assertLess(pts.shape[0], int(mask.sum()))
        self.assertGreater(pts.shape[0], 40)
        # 全ての輪郭点は円周付近(半径±3px)にある
        d = np.hypot(pts[:, 0] - 100, pts[:, 1] - 100)
        self.assertTrue(np.all(np.abs(d - 40) <= 3.0))

    def test_picks_largest_blob(self):
        # 大きい円と小さいノイズ円 → 大きい方の輪郭を返す
        img = draw_disk(blank(300, 300), 150, 150, 60, YELLOW_BGR)
        draw_disk(img, 30, 30, 8, YELLOW_BGR)
        pts = largest_contour_points(__import__("color").make_mask(img, YELLOW))
        d = np.hypot(pts[:, 0] - 150, pts[:, 1] - 150)
        self.assertTrue(np.all(np.abs(d - 60) <= 3.0))


class TestEstimateYellowBall(unittest.TestCase):
    def test_recovers_center_and_diameter(self):
        img = draw_disk(blank(480, 640), 320, 240, 80, YELLOW_BGR)
        res = estimate_yellow_ball(img, YELLOW, BallEstimationConfig(seed=0))
        self.assertIsNotNone(res)
        self.assertAlmostEqual(res.center_x, 320, delta=3)
        self.assertAlmostEqual(res.center_y, 240, delta=3)
        self.assertAlmostEqual(res.diameter, 160, delta=6)
        self.assertGreaterEqual(res.confidence, 0.0)
        self.assertLessEqual(res.confidence, 1.0)

    def test_robust_to_outliers(self):
        img = draw_disk(blank(480, 640), 300, 260, 70, YELLOW_BGR)
        # ボール以外の黄色ノイズ(外れ値)を散らす
        rng = np.random.default_rng(1)
        for _ in range(200):
            x = int(rng.integers(0, 640))
            y = int(rng.integers(0, 480))
            img[y, x] = YELLOW_BGR
        res = estimate_yellow_ball(img, YELLOW, BallEstimationConfig(seed=0))
        self.assertIsNotNone(res)
        self.assertAlmostEqual(res.center_x, 300, delta=6)
        self.assertAlmostEqual(res.center_y, 260, delta=6)

    def test_returns_none_when_no_ball(self):
        img = blank(480, 640)
        self.assertIsNone(estimate_yellow_ball(img, YELLOW, BallEstimationConfig(seed=0)))

    def test_returns_none_when_radius_out_of_range(self):
        img = draw_disk(blank(480, 640), 320, 240, 80, YELLOW_BGR)
        cfg = BallEstimationConfig(seed=0, max_radius_px=20.0)
        self.assertIsNone(estimate_yellow_ball(img, YELLOW, cfg))


if __name__ == "__main__":
    unittest.main()
