"""vision パッケージ共通のデータ型・設定。

vision_algorithm.md の推奨データ型(ColorRange / BallEstimationResult など)を
定義する。stdlib の `types` モジュールと名前が衝突しないよう、ファイル名は
`vision_types.py` としている(このディレクトリを sys.path[0] に載せて
`from color import ...` のように flat import するため)。

色範囲は OpenCV 互換の HSV 表現(H: 0-179, S: 0-255, V: 0-255)で持つ。
BGR→HSV 変換は color.bgr_to_hsv が担い、cv2 には依存しない。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass(frozen=True)
class ColorRange:
    """HSV の下限・上限(いずれも H:0-179, S:0-255, V:0-255)。

    赤のように色相が 0 度付近と 180 度付近に分かれる色は、複数の
    ColorRange を list で渡して OR を取る(color.make_mask 参照)。
    """

    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]


@dataclass(frozen=True)
class BallEstimationResult:
    """黄色ボールの推定結果。座標は入力画像のピクセル系(原点=左上)。"""

    center_x: float
    center_y: float
    diameter: float
    confidence: float  # RANSAC インライア率 0.0-1.0


@dataclass(frozen=True)
class BallEstimationConfig:
    """黄色ボール RANSAC 円フィットのパラメータ(全て解像度非依存で扱える)。

    モルフォロジー(ノイズ除去・穴埋め)の既定値は Twilight
    (robosweep_twilight/software/rpi/camera_py/ball_detect.py)を踏襲。
    """

    ransac_iters: int = 300            # 最大反復回数
    distance_tol_px: float = 3.0       # 円周からの最大許容誤差 [px]
    min_inlier_ratio: float = 0.3      # これ未満のインライア率は推定失敗
    min_radius_px: float = 10.0        # 有効半径の下限
    max_radius_px: Optional[float] = None  # 有効半径の上限(None=無制限)
    max_points: int = 2000             # RANSAC に入れる輪郭点の上限(サンプリング)
    min_contour_points: int = 30       # 最大輪郭の点数がこれ未満なら推定失敗
    morph_kernel_px: int = 7           # モルフォロジーの楕円カーネル径(0で無効)
    morph_close_iters: int = 2         # 穴埋め(CLOSE)反復
    morph_open_iters: int = 1          # ノイズ除去(OPEN)反復
    seed: Optional[int] = None         # 乱数シード(再現性が必要なら指定)


@dataclass(frozen=True)
class WallEdgeDetectionConfig:
    """赤色壁上面の下端エッジ検出パラメータ。"""

    cluster_gap_px: float = 20.0       # 下端点群を行方向で分割するギャップ
    min_region_points: int = 15        # クラスタ(壁領域)の最小点数
    min_column_span_px: float = 20.0   # 選んだエッジが跨ぐ列幅の下限(垂直線ガード)
    ransac_iters: int = 100            # 直線 RANSAC の反復回数
    distance_tol_px: float = 3.0       # 直線からの最大許容誤差 [px]
    min_inlier_points: int = 10        # 直線インライアの最小点数
    morph_kernel_px: int = 5           # 赤マスクのノイズ除去カーネル径(0で無効)
    seed: Optional[int] = None


# --- 既定の色範囲(実機の照明で要調整。vision/README.md 参照) ---
# 黄色ボール: Twilight の実機チューニング値を踏襲(V 上限 200 で白飛びを除外、
# S 下限 150 で淡い黄を排除)。robosweep_twilight/.../ball_detect.py 参照。
DEFAULT_YELLOW: ColorRange = ColorRange(lower=(15, 150, 80), upper=(36, 255, 200))

# 赤色壁上面: H が 0 付近と 180 付近に分かれるため 2 範囲。
DEFAULT_RED: Tuple[ColorRange, ...] = (
    ColorRange(lower=(0, 80, 60), upper=(10, 255, 255)),
    ColorRange(lower=(160, 80, 60), upper=(179, 255, 255)),
)
