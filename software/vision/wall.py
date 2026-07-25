"""赤色の壁上面(マイクロマウス)の「下端」エッジ検出。

vision_algorithm.md §10-15, §23 の制約に従う:
- 赤色領域の「下端境界」(各列で赤が現れる最大 y)を抽出する。上端や
  領域内部のエッジではない(§23.3, §23.4)。
- 複数の赤色領域があれば各領域の下端候補を作り、画像内で最も下側
  (=最も手前)の候補を選ぶ(§23.5)。
- 直線フィットは局所ノイズ・マスク欠損に強い RANSAC を使う(§23.6)。
- 結果は y = a*x + b の (a, b) で返す(§23.7)。x=列, y=行。
- 垂直に近い(列幅が狭い)エッジは表現できないので検出失敗扱い(§15)。

なお software/micromouse/vision.py は同じ赤帯の「上端」をジャイロ角度補正の
ために検出する別モジュール。用途(下端 vs 上端)が逆なので統合していない。
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from color import clean_mask, make_mask
from vision_types import ColorRange, WallEdgeDetectionConfig

ColorSpec = "ColorRange | list[ColorRange] | tuple[ColorRange, ...]"


def lower_edge_points(mask: np.ndarray) -> np.ndarray:
    """赤マスクの各列について、赤が現れる最大 y(下端)を点 (x, y) で返す。

    赤が 1 画素も無い列は除外。戻り値 (N,2) float64。
    """
    h, w = mask.shape
    row_idx = np.arange(h, dtype=np.int32)[:, None]
    masked_rows = np.where(mask, row_idx, -1)
    max_row = masked_rows.max(axis=0)          # 列ごとの下端行(赤無しは -1)
    cols = np.flatnonzero(max_row >= 0)
    if cols.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.column_stack([cols, max_row[cols]]).astype(np.float64)


def cluster_by_row(
    points: np.ndarray, *, gap_px: float, min_points: int
) -> List[np.ndarray]:
    """下端点群を行方向のギャップで分割する簡易 1 次元クラスタリング。

    行の平均が大きい(=画像下側=手前)順にソートして返す。
    """
    if points.shape[0] == 0:
        return []
    order = np.argsort(points[:, 1])
    sorted_pts = points[order]
    gaps = np.diff(sorted_pts[:, 1])
    split = np.flatnonzero(gaps > gap_px) + 1
    groups = np.split(sorted_pts, split)
    clusters = [g for g in groups if g.shape[0] >= min_points]
    clusters.sort(key=lambda g: g[:, 1].mean(), reverse=True)
    return clusters


def _ransac_line(
    points: np.ndarray, cfg: WallEdgeDetectionConfig
) -> Optional[Tuple[float, float]]:
    """点群 (x, y) に y = a*x + b を RANSAC でフィット → (a, b)。"""
    n = points.shape[0]
    if n < 2:
        return None
    x = points[:, 0]
    y = points[:, 1]
    rng = np.random.default_rng(cfg.seed)

    best_inliers: Optional[np.ndarray] = None
    best_count = 0
    for _ in range(cfg.ransac_iters):
        i, j = rng.choice(n, size=2, replace=False)
        dx = x[j] - x[i]
        if abs(dx) < 1e-9:
            continue
        a = (y[j] - y[i]) / dx
        b = y[i] - a * x[i]
        resid = np.abs(y - (a * x + b))
        inliers = resid < cfg.distance_tol_px
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < cfg.min_inlier_points:
        return None

    a, b = np.polyfit(x[best_inliers], y[best_inliers], 1)
    return float(a), float(b)


def detect_nearest_red_wall_edge(
    image: np.ndarray,
    color_spec,
    config: WallEdgeDetectionConfig = WallEdgeDetectionConfig(),
) -> Optional[Tuple[float, float]]:
    """最も手前(画像で最も下側)の赤壁上面の下端エッジ y = a*x + b を返す。

    color_spec は赤の ColorRange 単体または複数(色相ラップ対応)。
    検出失敗(赤が無い・点数不足・垂直に近い・フィット不能)なら None。
    """
    mask = make_mask(image, color_spec)
    # OPEN のみ(赤の孤立ノイズが偽の下端点を作るのを防ぐ)。穴埋め CLOSE は
    # 下端位置を下方向にずらしうるので使わない。
    mask = clean_mask(mask, kernel_px=config.morph_kernel_px, close_iters=0, open_iters=1)
    pts = lower_edge_points(mask)
    clusters = cluster_by_row(
        pts, gap_px=config.cluster_gap_px, min_points=config.min_region_points
    )
    if not clusters:
        return None

    target = clusters[0]  # 最も下側(手前)
    col_span = target[:, 0].max() - target[:, 0].min()
    if col_span < config.min_column_span_px:  # 垂直線ガード
        return None

    return _ransac_line(target, config)
