"""黄色ボールの検出(高速 bool 判定)と中心・直径推定(RANSAC 円フィット)。

vision_algorithm.md の制約に従う:
- 検出(detect_yellow_ball)は黄色ピクセル割合のしきい値のみ。輪郭抽出・
  RANSAC はしない(高速前段処理)。
- 推定(estimate_yellow_ball)は「黄色マスク全画素」ではなく「輪郭点」を
  RANSAC 円フィットの入力にする(§23.1)。信頼度=インライア率(§23.2)。
- マスク生成は cv2(color.py)、輪郭抽出・RANSAC は numpy。
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from color import clean_mask, make_mask, mask_ratio
from vision_types import BallEstimationConfig, BallEstimationResult, ColorRange


def detect_yellow_ball(
    image: np.ndarray, color_range: ColorRange, threshold: float
) -> bool:
    """黄色ボールが存在する可能性が高いかを高速判定する。

    黄色ピクセルの割合が threshold を超えたら True。解像度に依存しない。
    """
    mask = make_mask(image, color_range)
    return mask_ratio(mask) > threshold


def largest_contour_points(mask: np.ndarray) -> np.ndarray:
    """bool マスクの最大外部輪郭の点列 (x, y) を (N,2) float64 で返す。

    cv2.findContours(RETR_EXTERNAL) で外部輪郭を取り、面積最大の 1 つを選ぶ
    (Twilight の ball_detect.py と同方針)。ボール以外の小さな黄色ノイズ
    ブロブを除外でき、その輪郭点だけを RANSAC 円フィットに渡せる。
    輪郭が無ければ空 (0,2)。
    """
    m = (mask.astype(np.uint8)) * 255
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.empty((0, 2), dtype=np.float64)
    cnt = max(contours, key=cv2.contourArea)
    return cnt.reshape(-1, 2).astype(np.float64)


def _circle_from_3_points(p: np.ndarray) -> Optional[tuple]:
    """3 点 (3,2) を通る円の (cx, cy, r)。3 点が同一直線上なら None。"""
    (ax, ay), (bx, by), (cx, cy) = p
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    r = float(np.hypot(ax - ux, ay - uy))
    return float(ux), float(uy), r


def _fit_circle_algebraic(points: np.ndarray) -> Optional[tuple]:
    """点群 (N,2) に対する Kasa 法(代数的最小二乗)円フィット → (cx, cy, r)。"""
    x = points[:, 0]
    y = points[:, 1]
    a_mat = np.column_stack([x, y, np.ones_like(x)])
    b_vec = x * x + y * y
    sol, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
    cx = sol[0] / 2.0
    cy = sol[1] / 2.0
    val = sol[2] + cx * cx + cy * cy
    if val < 0:
        return None
    return float(cx), float(cy), float(np.sqrt(val))


def estimate_yellow_ball(
    image: np.ndarray,
    color_range: ColorRange,
    config: BallEstimationConfig = BallEstimationConfig(),
) -> Optional[BallEstimationResult]:
    """黄色領域の輪郭点を RANSAC 円フィットしてボール中心・直径を推定する。

    推定できなければ None(輪郭点不足・有効円なし・半径範囲外・
    インライア率不足)。
    """
    mask = make_mask(image, color_range)
    mask = clean_mask(
        mask,
        kernel_px=config.morph_kernel_px,
        close_iters=config.morph_close_iters,
        open_iters=config.morph_open_iters,
    )
    pts = largest_contour_points(mask)
    if pts.shape[0] < config.min_contour_points:
        return None

    rng = np.random.default_rng(config.seed)

    # 点が多すぎる場合はサンプリング(FullHD 対策)。信頼度の分母にもなる。
    if pts.shape[0] > config.max_points:
        sel = rng.choice(pts.shape[0], size=config.max_points, replace=False)
        sample = pts[sel]
    else:
        sample = pts
    n = sample.shape[0]

    best_inliers: Optional[np.ndarray] = None
    best_count = 0
    for _ in range(config.ransac_iters):
        idx = rng.choice(n, size=3, replace=False)
        circle = _circle_from_3_points(sample[idx])
        if circle is None:
            continue
        cx, cy, r = circle
        if r < config.min_radius_px:
            continue
        if config.max_radius_px is not None and r > config.max_radius_px:
            continue
        dist = np.abs(np.hypot(sample[:, 0] - cx, sample[:, 1] - cy) - r)
        inliers = dist < config.distance_tol_px
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 3:
        return None

    confidence = best_count / n
    if confidence < config.min_inlier_ratio:
        return None

    refined = _fit_circle_algebraic(sample[best_inliers])
    if refined is None:
        return None
    cx, cy, r = refined
    if r < config.min_radius_px:
        return None
    if config.max_radius_px is not None and r > config.max_radius_px:
        return None

    return BallEstimationResult(
        center_x=cx, center_y=cy, diameter=2.0 * r, confidence=confidence
    )
