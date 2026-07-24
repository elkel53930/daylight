import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision import (
    cluster_by_row,
    detect_red_band_top_edge,
    fit_top_edge,
    red_mask,
    select_nearest_cluster,
    top_edge_points,
)

WHITE = (200, 200, 200)
RED = (230, 70, 90)
BLACK = (20, 20, 20)


def make_image(h: int, w: int, top_edge_row_at, cols=None) -> np.ndarray:
    """指定した上端エッジ(列->行の関数)を持つ合成画像を作る。

    行 < top_edge_row: 白壁面 / 行 >= top_edge_row: 赤帯(帯の高さ30px)。
    cols を指定すると、その列範囲にだけ帯を描く。
    """
    img = np.full((h, w, 3), WHITE, dtype=np.uint8)
    add_band(img, top_edge_row_at, cols=cols)
    return img


def add_band(img: np.ndarray, top_edge_row_at, band_height: int = 30, cols=None) -> None:
    """既存の画像に、指定した上端エッジを持つ赤帯をもう1本描き足す(in-place)。

    cols を指定すると、その列範囲にだけ描く(手前の壁が写る範囲と奥の壁が
    写る範囲が列方向でも別、という実機画像に近い状況を作るため)。
    """
    h, w, _ = img.shape
    col_range = cols if cols is not None else range(w)
    for col in col_range:
        top = int(round(top_edge_row_at(col)))
        bottom = min(h, top + band_height)
        if 0 <= top < h:
            img[top:bottom, col] = RED


class TestRedMask(unittest.TestCase):
    def test_flat_band(self):
        img = make_image(200, 300, lambda col: 100)
        mask = red_mask(img)
        self.assertTrue(mask[100, 150])
        self.assertFalse(mask[50, 150])

    def test_no_red(self):
        img = np.full((50, 50, 3), WHITE, dtype=np.uint8)
        mask = red_mask(img)
        self.assertFalse(mask.any())


class TestTopEdgePoints(unittest.TestCase):
    def test_flat_band_all_columns(self):
        img = make_image(200, 300, lambda col: 100)
        points = top_edge_points(red_mask(img))
        self.assertEqual(points.shape[0], 300)
        self.assertTrue(np.all(points[:, 1] == 100))

    def test_empty_when_no_red(self):
        img = np.full((50, 50, 3), WHITE, dtype=np.uint8)
        points = top_edge_points(red_mask(img))
        self.assertEqual(points.shape[0], 0)

    def test_two_runs_in_same_column_both_returned(self):
        """実機画像で確認された不具合の再現テスト: 同じ列に奥の壁(上)と
        手前の壁(下)の赤帯が両方写っている場合、両方の上端を拾えること
        (「最初に現れる行」だけだと手前の壁が丸ごと欠落していた)。
        """
        img = make_image(700, 300, lambda col: 500)  # 手前(下)
        add_band(img, lambda col: 150, band_height=10)  # 奥(上、同じ列全体に重ねて描く)
        points = top_edge_points(red_mask(img))
        # 各列につき2点(奥:150, 手前:500)が返るはず
        self.assertEqual(points.shape[0], 600)
        rows_at_col0 = sorted(points[points[:, 0] == 0][:, 1])
        self.assertEqual(rows_at_col0, [150.0, 500.0])


class TestFitTopEdge(unittest.TestCase):
    def test_flat_edge_recovers_slope_zero(self):
        img = make_image(200, 300, lambda col: 100)
        edge = detect_red_band_top_edge(img)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, 0.0, places=3)
        self.assertAlmostEqual(edge.intercept, 100.0, places=1)

    def test_tilted_edge_recovers_slope(self):
        true_slope = 0.05  # 画像右へ行くほど下がる(ヨー誤差相当)
        true_intercept = 80.0
        img = make_image(200, 400, lambda col: true_intercept + true_slope * col)
        edge = detect_red_band_top_edge(img)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, true_slope, places=3)
        self.assertAlmostEqual(edge.intercept, true_intercept, places=0)

    def test_outlier_blob_is_rejected(self):
        """リモコン等の無関係な赤色誤検出があってもフィットを乱されないこと。"""
        img = make_image(200, 400, lambda col: 100)
        # 画面端に無関係な赤いブロブ(別の高さ)を追加
        img[10:30, 5:25] = RED
        edge = detect_red_band_top_edge(img)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, 0.0, places=2)
        self.assertAlmostEqual(edge.intercept, 100.0, places=0)

    def test_no_red_returns_none(self):
        img = np.full((50, 50, 3), WHITE, dtype=np.uint8)
        self.assertIsNone(detect_red_band_top_edge(img))

    def test_too_few_points_returns_none(self):
        img = np.full((50, 50, 3), WHITE, dtype=np.uint8)
        img[10, 10] = RED  # 1画素だけ
        self.assertIsNone(fit_top_edge(top_edge_points(red_mask(img))))


class TestClusterByRow(unittest.TestCase):
    def test_single_band_one_cluster(self):
        img = make_image(200, 300, lambda col: 100)
        points = top_edge_points(red_mask(img))
        clusters = cluster_by_row(points)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(clusters[0].shape[0], 300)

    def test_two_bands_split_into_two_clusters(self):
        # 実機画像同様、奥の壁(row小)が写る列範囲と手前の壁(row大)が
        # 写る列範囲が別々になっているケース(手前の壁が写る列では、
        # 奥の壁はそもそも視野に入っていない)。
        img = make_image(700, 300, lambda col: 150, cols=range(0, 100))  # 奥
        add_band(img, lambda col: 500, cols=range(100, 300))  # 手前
        points = top_edge_points(red_mask(img))
        clusters = cluster_by_row(points)
        self.assertEqual(len(clusters), 2)
        # 先頭(手前=行が大きい方)が near cluster であること
        self.assertAlmostEqual(clusters[0][:, 1].mean(), 500.0, delta=1.0)
        self.assertAlmostEqual(clusters[1][:, 1].mean(), 150.0, delta=1.0)

    def test_tiny_cluster_is_discarded(self):
        img = make_image(700, 300, lambda col: 150)
        # 5列だけの小さなノイズ的ブロブ
        img[400:420, 10:15] = RED
        points = top_edge_points(red_mask(img))
        clusters = cluster_by_row(points, min_points=20)
        self.assertEqual(len(clusters), 1)

    def test_empty_points(self):
        self.assertEqual(cluster_by_row(np.zeros((0, 2))), [])


class TestDetectRedBandTopEdgeMultiBand(unittest.TestCase):
    def test_selects_nearest_of_two_bands(self):
        far_slope, far_intercept = 0.01, 150.0
        near_slope, near_intercept = -0.02, 500.0
        img = make_image(700, 400, lambda col: far_intercept + far_slope * col, cols=range(0, 150))
        add_band(img, lambda col: near_intercept + near_slope * col, cols=range(150, 400))

        edge = detect_red_band_top_edge(img, cluster_select=select_nearest_cluster)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, near_slope, places=2)
        self.assertAlmostEqual(edge.intercept, near_intercept, places=0)

    def test_can_select_farthest_cluster_explicitly(self):
        far_slope, far_intercept = 0.01, 150.0
        near_slope, near_intercept = -0.02, 500.0
        img = make_image(700, 400, lambda col: far_intercept + far_slope * col, cols=range(0, 150))
        add_band(img, lambda col: near_intercept + near_slope * col, cols=range(150, 400))

        def select_farthest(clusters):
            return clusters[-1] if clusters else None

        edge = detect_red_band_top_edge(img, cluster_select=select_farthest)
        self.assertIsNotNone(edge)
        self.assertAlmostEqual(edge.slope, far_slope, places=2)
        self.assertAlmostEqual(edge.intercept, far_intercept, places=0)


if __name__ == "__main__":
    unittest.main()
