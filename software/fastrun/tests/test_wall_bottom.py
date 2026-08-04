"""wall_bottom(赤壁 下端エッジ検出アダプタ)のテスト(合成画像、cv2使用)。"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from wall_bottom import detect_red_band_bottom_edge

RED_RGB = (255, 0, 0)  # RGB(detect側でBGRへ反転される)


def make_band(w, h, slope, top, thickness=80):
    """上端が row=slope*col+top の赤帯を持つ合成RGB画像。下端≈top+thickness。"""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    for col in range(w):
        t = int(round(slope * col + top))
        t = max(0, min(h - 1, t))
        img[t:min(h, t + thickness), col] = RED_RGB
    return img


class TestBottomEdge(unittest.TestCase):
    def test_detects_bottom_not_top(self):
        # 上端200・厚80 → 下端≈280。下端エッジなので intercept は280側(上端200ではない)。
        e = detect_red_band_bottom_edge(make_band(400, 600, 0.0, top=200, thickness=80))
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e.intercept, 280, delta=6)
        self.assertAlmostEqual(e.slope, 0.0, delta=0.02)

    def test_tilted(self):
        e = detect_red_band_bottom_edge(make_band(400, 600, 0.1, top=150, thickness=60))
        self.assertIsNotNone(e)
        self.assertAlmostEqual(e.slope, 0.1, delta=0.02)

    def test_no_red_returns_none(self):
        self.assertIsNone(detect_red_band_bottom_edge(np.zeros((200, 200, 3), np.uint8)))

    def test_nearest_of_two_bands(self):
        # 手前(下)の壁と奥(上)の壁 → 手前(下端が大きい方)を選ぶ。
        img = make_band(400, 700, 0.0, top=120, thickness=40)   # 奥(上)
        near = make_band(400, 700, 0.0, top=480, thickness=60)  # 手前(下)
        img[near.any(axis=2)] = RED_RGB
        e = detect_red_band_bottom_edge(img)
        self.assertIsNotNone(e)
        self.assertGreater(e.intercept, 400)  # 手前(下=intercept大)を選ぶ


if __name__ == "__main__":
    unittest.main()
