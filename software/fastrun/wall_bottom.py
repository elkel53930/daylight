"""wall_bottom.py — 赤壁の「下端」エッジ検出アダプタ(2026-08-04)。

現行の上端エッジ(vision_wall.detect_red_band_top_edge)は、赤い壁上面と迷路の
「外側の背景」の境界を見るため、背景に制御できない赤色物体があると汚染される。
壁の**下端エッジ**(赤い上面と白い壁面=迷路内側の境界)は迷路内で完結するので
背景汚染に強い(ユーザー方針、2026-08-04)。

下端検出アルゴリズムは既存の software/vision/wall.py(HSV+RANSAC)を再利用し、
ここで現行 RedBandEdge(slope=a, intercept=b, inlier_count, residual_std)互換の
形に包む。これにより camera_align.estimate_pose / recenter.* の検出器を差し替え
られる。

⚠️ 下端は上端より画像で下(row大)にあるので、上端で取った較正定数
(CAMERA_ROW_AT_90MM=689.9 / CAMERA_ROW_PX_PER_MM=5.38 / ヨーの slope gain)は
全て無効。**下端エッジ用に実機で再較正してから**位置・角度補正に使うこと。
⚠️ 角(コーナー)では下端も∧形になり得る点は上端と同じ(別課題、neighbor_for_axis)。
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np

# software/vision/ を import パスへ(color/wall/vision_types はそこにある)。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "vision"))

from color import clean_mask, make_mask  # noqa: E402
from vision_types import DEFAULT_RED, WallEdgeDetectionConfig  # noqa: E402
from wall import cluster_by_row, lower_edge_points  # noqa: E402 (vision/wall.py)

from vision_wall import RedBandEdge  # 現行と同じ返り値型を使う


def _fit_line_with_stats(points: np.ndarray, cfg: WallEdgeDetectionConfig):
    """(x,y)点群に y=a*x+b を RANSAC フィットし (a,b,inlier_count,residual_std) を返す。

    vision/wall._ransac_line と同じ手順だが、信頼度ゲート用に残差stdとインライア数も返す。
    """
    n = points.shape[0]
    if n < 2:
        return None
    x, y = points[:, 0], points[:, 1]
    rng = np.random.default_rng(cfg.seed)
    best_inliers = None
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
    res = y[best_inliers] - (a * x[best_inliers] + b)
    return float(a), float(b), int(best_inliers.sum()), float(res.std())


def detect_red_band_bottom_edge(
    img_rgb: np.ndarray, *, config: Optional[WallEdgeDetectionConfig] = None
) -> Optional[RedBandEdge]:
    """RGB画像から最も手前の赤壁の「下端」エッジを RedBandEdge 互換で返す。

    現行 detect_red_band_top_edge と同じインターフェース(row = slope*col + intercept)。
    検出失敗(赤なし/点数不足/垂直に近い/フィット不能)なら None。
    ⚠️ make_mask は BGR 前提。OnboardCamera.capture() は RGB を返すので反転して渡す。
    """
    cfg = config or WallEdgeDetectionConfig()
    image_bgr = np.ascontiguousarray(img_rgb[:, :, ::-1])
    mask = make_mask(image_bgr, DEFAULT_RED)
    # OPEN のみ(孤立ノイズ除去)。CLOSE は下端を下方向へずらしうるので使わない。
    mask = clean_mask(mask, kernel_px=cfg.morph_kernel_px, close_iters=0, open_iters=1)
    pts = lower_edge_points(mask)
    clusters = cluster_by_row(pts, gap_px=cfg.cluster_gap_px,
                              min_points=cfg.min_region_points)
    if not clusters:
        return None
    target = clusters[0]  # 最も下側=手前
    if target[:, 0].max() - target[:, 0].min() < cfg.min_column_span_px:
        return None  # 垂直線ガード
    fit = _fit_line_with_stats(target, cfg)
    if fit is None:
        return None
    a, b, inliers, res_std = fit
    return RedBandEdge(slope=a, intercept=b, inlier_count=inliers, residual_std=res_std)
