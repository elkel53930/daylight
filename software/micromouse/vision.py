"""カメラ画像からの壁上面(赤帯)エッジ検出。

マイクロマウス競技規則により壁上面は赤色に塗られている。前方固定カメラ
(software/arm/futaba_servo.py で角度0固定、hw_test.py camera-capture 参照)
で撮った画像には、壁の白い側面と赤い上面の境界が「赤色の帯」として写る。
帯には上下2本のエッジがあるが、機体の角度・位置補正には上側のエッジ
(壁上面の奥側の境界)のみを使う方針(2026-07-24)。

画像中の各列について、赤帯マスクで最初に現れる行(=上端)を拾い、その
点群に直線をロバストフィットする。フィットで得た傾き(slope)・切片
(intercept)から、機体の角度・位置ズレを推定するのは呼び出し側の役目
(このモジュールは画像処理のみを担当し、迷路・制御ロジックには依存しない)。

実機画像(logs/camera/latest.jpg, 2026-07-24)で確認済みの通り、手前の壁と
奥の壁の赤帯が同時に写り込むことがある。この場合、上端の点群は行の値で
複数のかたまり(クラスタ)に分かれる(遠いほど画像上部=行が小さい)。
1本の直線で全部をまとめてフィットすると意味のない結果になるため、まず
行方向でクラスタリングしてから、目的のクラスタ(既定: 最も手前=画像下側)
だけをフィット対象に選ぶ。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional

import numpy as np

# 赤帯しきい値(RGB)。hw_test.py camera-capture で撮った実機画像
# (software/micromouse/logs/camera/latest.jpg, 2026-07-24)で確認済み:
# 赤帯 R≈230/G≈70-100/B≈70-100、それ以外(白壁面・黒床・背景)は
# R-G, R-B が十分小さい。個体差・照明条件が変わったら実機画像で見直すこと。
RED_R_MIN = 140
RED_R_MINUS_G_MIN = 50
RED_R_MINUS_B_MIN = 30

# ロバストフィットの外れ値除去(リモコン等の無関係な赤色誤検出を弾く)
FIT_MAX_ITERS = 5
FIT_SIGMA_CLIP = 2.5
FIT_MIN_POINTS = 20

# 行方向クラスタリング(手前/奥など複数の壁の赤帯を分離する)。
# 実機画像では手前の壁(row≈500-640)と奥の壁(row≈150-300)の間に
# 200px近い隙間があったため、同一クラスタ内のジャギー(数px〜十数px)
# より十分大きく、かつ別の壁を誤って混ぜない値として30pxとした。
CLUSTER_GAP_PX = 30.0
CLUSTER_MIN_POINTS = FIT_MIN_POINTS


@dataclass(frozen=True)
class RedBandEdge:
    """赤帯上端エッジの直線フィット結果。

    画像座標系(右方向+col、下方向+row)で `row = slope * col + intercept`。
    """

    slope: float
    intercept: float
    inlier_count: int
    residual_std: float


def red_mask(img: np.ndarray) -> np.ndarray:
    """RGB画像(H,W,3 uint8)から赤帯マスク(H,W bool)を作る。"""
    r = img[:, :, 0].astype(np.int16)
    g = img[:, :, 1].astype(np.int16)
    b = img[:, :, 2].astype(np.int16)
    return (r > RED_R_MIN) & (r - g > RED_R_MINUS_G_MIN) & (r - b > RED_R_MINUS_B_MIN)


def top_edge_points(mask: np.ndarray) -> np.ndarray:
    """各列にある赤の連続区間(run)ごとに、その上端の行を点として返す。

    1列に手前の壁と奥の壁の赤帯が両方写っている場合など、赤の区間が
    複数あることがある(実機画像で確認済み、2026-07-24)。区間ごとに
    別の点として拾わないと、彩度の低い遠い壁の赤が手前の壁より上に
    たまたま写っただけで、近い壁の赤帯が点群から丸ごと欠落してしまう。
    そのため「最初に現れる行」ではなく、非赤→赤の立ち上がり全てを拾う。
    赤が無い列は除外する。戻り値は shape (N, 2) の float64 配列
    (N=0 なら赤が1画素も無い)。
    """
    h, w = mask.shape
    cols = []
    rows = []
    for col in range(w):
        col_mask = mask[:, col]
        if not col_mask.any():
            continue
        diff = np.diff(col_mask.astype(np.int8))
        starts = np.flatnonzero(diff == 1) + 1
        if col_mask[0]:
            starts = np.concatenate(([0], starts))
        for row in starts:
            cols.append(col)
            rows.append(row)
    return np.column_stack([cols, rows]).astype(np.float64)


def fit_top_edge(
    points: np.ndarray,
    *,
    max_iters: int = FIT_MAX_ITERS,
    sigma_clip: float = FIT_SIGMA_CLIP,
    min_points: int = FIT_MIN_POINTS,
) -> Optional[RedBandEdge]:
    """(col, row)点群に直線をロバストフィットする。

    残差が sigma_clip * 標準偏差を超える点を繰り返し除去してから
    再フィットする、簡易的な反復的外れ値除去(IRLS の簡易版)。
    点が少なすぎる・収束しない場合は None。
    """
    if points.shape[0] < min_points:
        return None

    cols, rows = points[:, 0], points[:, 1]
    inlier_mask = np.ones(cols.shape[0], dtype=bool)
    for _ in range(max_iters):
        if inlier_mask.sum() < min_points:
            return None
        slope, intercept = np.polyfit(cols[inlier_mask], rows[inlier_mask], 1)
        all_residuals = rows - (slope * cols + intercept)
        std = all_residuals[inlier_mask].std()
        if std < 1e-6:
            break
        new_mask = np.abs(all_residuals) < sigma_clip * std
        if np.array_equal(new_mask, inlier_mask):
            break
        inlier_mask = new_mask

    if inlier_mask.sum() < min_points:
        return None

    slope, intercept = np.polyfit(cols[inlier_mask], rows[inlier_mask], 1)
    residuals = rows[inlier_mask] - (slope * cols[inlier_mask] + intercept)
    return RedBandEdge(
        slope=float(slope),
        intercept=float(intercept),
        inlier_count=int(inlier_mask.sum()),
        residual_std=float(residuals.std()),
    )


def cluster_by_row(
    points: np.ndarray,
    *,
    gap_px: float = CLUSTER_GAP_PX,
    min_points: int = CLUSTER_MIN_POINTS,
) -> List[np.ndarray]:
    """(col, row)点群を行方向のギャップでクラスタリングする。

    行でソートし、隣接する点の行の差が gap_px を超える箇所で分割する
    簡易的な1次元クラスタリング。min_points 未満の小さなクラスタ
    (ノイズ的な誤検出)は捨てる。戻り値は行の平均値が大きい
    (=画像下側=手前)順にソートされたクラスタのリスト。
    """
    if points.shape[0] == 0:
        return []

    order = np.argsort(points[:, 1])
    sorted_points = points[order]
    rows = sorted_points[:, 1]

    gaps = np.diff(rows)
    split_indices = np.flatnonzero(gaps > gap_px) + 1
    groups = np.split(sorted_points, split_indices)

    clusters = [g for g in groups if g.shape[0] >= min_points]
    clusters.sort(key=lambda g: g[:, 1].mean(), reverse=True)
    return clusters


def select_nearest_cluster(clusters: List[np.ndarray]) -> Optional[np.ndarray]:
    """最も画像下側(=手前)のクラスタを選ぶ。無ければNone。"""
    return clusters[0] if clusters else None


def detect_red_band_top_edge(
    img: np.ndarray,
    *,
    cluster_select: Callable[[List[np.ndarray]], Optional[np.ndarray]] = select_nearest_cluster,
) -> Optional[RedBandEdge]:
    """画像から壁上面(赤帯)の上端エッジを検出する。

    複数の壁の赤帯が同時に写っている場合は行方向でクラスタリングし、
    cluster_select が選んだ1クラスタのみをフィット対象にする
    (既定: 最も手前=画像下側のクラスタ)。検出できなければNone。
    """
    mask = red_mask(img)
    points = top_edge_points(mask)
    clusters = cluster_by_row(points)
    target = cluster_select(clusters)
    if target is None:
        return None
    return fit_top_edge(target)
