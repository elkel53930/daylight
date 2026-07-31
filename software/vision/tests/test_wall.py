import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from wall import (
    cluster_by_row,
    detect_nearest_red_wall_edge,
    lower_edge_points,
)
from vision_types import DEFAULT_RED, WallEdgeDetectionConfig

RED_BGR = (0, 0, 255)


def blank(h, w):
    return np.zeros((h, w, 3), dtype=np.uint8)


def draw_band(img, a, b, thickness, bgr):
    """y = a*x + b を下端(帯の下側境界)とする厚み thickness の赤帯を描く。"""
    h, w = img.shape[:2]
    for x in range(w):
        y_bottom = int(round(a * x + b))
        y_top = y_bottom - thickness
        y0 = max(0, y_top)
        y1 = min(h, y_bottom + 1)
        if y0 < y1:
            img[y0:y1, x] = bgr
    return img


class TestLowerEdgePoints(unittest.TestCase):
    def test_extracts_bottom_row_per_column(self):
        img = draw_band(blank(200, 100), 0.0, 120, 10, RED_BGR)
        from color import make_mask
        mask = make_mask(img, DEFAULT_RED)
        pts = lower_edge_points(mask)
        # 各列の下端はほぼ y=120
        self.assertEqual(pts.shape[0], 100)
        self.assertTrue(np.all(np.abs(pts[:, 1] - 120) <= 1))


class TestClustering(unittest.TestCase):
    def test_two_bands_split_and_sorted_nearest_first(self):
        pts = np.array(
            [[x, 100] for x in range(30)] + [[x, 300] for x in range(30)],
            dtype=np.float64,
        )
        clusters = cluster_by_row(pts, gap_px=20.0, min_points=15)
        self.assertEqual(len(clusters), 2)
        # 先頭が手前(行の平均が大きい=300 側)
        self.assertAlmostEqual(clusters[0][:, 1].mean(), 300)
        self.assertAlmostEqual(clusters[1][:, 1].mean(), 100)


class TestDetectNearestRedWallEdge(unittest.TestCase):
    def test_single_horizontal_edge(self):
        img = draw_band(blank(300, 200), 0.0, 150, 12, RED_BGR)
        res = detect_nearest_red_wall_edge(img, DEFAULT_RED, WallEdgeDetectionConfig(seed=0))
        self.assertIsNotNone(res)
        a, b = res
        self.assertAlmostEqual(a, 0.0, delta=0.02)
        self.assertAlmostEqual(b, 150, delta=2)

    def test_sloped_edge(self):
        img = draw_band(blank(400, 300), 0.1, 100, 12, RED_BGR)
        res = detect_nearest_red_wall_edge(img, DEFAULT_RED, WallEdgeDetectionConfig(seed=0))
        self.assertIsNotNone(res)
        a, b = res
        self.assertAlmostEqual(a, 0.1, delta=0.02)
        self.assertAlmostEqual(b, 100, delta=3)

    def test_selects_lower_of_two_bands(self):
        # 上側 y=0.1x+100、下側 y=0.1x+300 → 下側(手前)を選ぶ
        img = blank(500, 400)
        draw_band(img, 0.1, 100, 12, RED_BGR)
        draw_band(img, 0.1, 300, 12, RED_BGR)
        res = detect_nearest_red_wall_edge(img, DEFAULT_RED, WallEdgeDetectionConfig(seed=0))
        self.assertIsNotNone(res)
        a, b = res
        self.assertAlmostEqual(b, 300, delta=4)

    def test_returns_none_without_red(self):
        img = blank(300, 200)
        self.assertIsNone(
            detect_nearest_red_wall_edge(img, DEFAULT_RED, WallEdgeDetectionConfig(seed=0))
        )

    def test_vertical_guard_rejects_narrow_span(self):
        # 数列だけの赤(列幅が狭い)は垂直線ガードで None
        img = blank(300, 200)
        img[50:250, 100:104] = RED_BGR
        cfg = WallEdgeDetectionConfig(seed=0, min_column_span_px=20.0, min_region_points=15)
        self.assertIsNone(detect_nearest_red_wall_edge(img, DEFAULT_RED, cfg))


if __name__ == "__main__":
    unittest.main()
