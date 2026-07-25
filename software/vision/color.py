"""色空間変換と HSV マスク生成(OpenCV/cv2 使用)。

Picamera2 から取得する BGR 画像(H,W,3 uint8)を対象とする。HSV の範囲は
OpenCV 準拠(H:0-179, S:0-255, V:0-255)なので、実機で撮った画像を GIMP 等で
調べた HSV 値をそのまま ColorRange に使える。

vision_algorithm.md の要求どおり:
- 色判定は RGB 値の直接比較ではなく HSV 範囲で行う
- 解像度をハードコードしない
- Python のピクセルループを使わない(cv2 と numpy のベクトル化のみ)

cv2(python3-opencv)は Raspberry Pi では apt で導入する。venv は
--system-site-packages なので apt で入れれば見える(vision/README.md 参照)。
"""

from __future__ import annotations

from typing import Iterable, Sequence, Union

import cv2
import numpy as np

from vision_types import ColorRange


def bgr_to_hsv(image: np.ndarray) -> np.ndarray:
    """BGR(H,W,3 uint8)→ HSV(H,W,3 uint8, OpenCV レンジ H:0-179)。"""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"BGR画像 (H,W,3) を渡してください: shape={image.shape}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def in_range(hsv: np.ndarray, color_range: ColorRange) -> np.ndarray:
    """HSV 画像から 1 つの ColorRange に該当する bool マスク(H,W)を返す。"""
    lo = np.asarray(color_range.lower, dtype=np.uint8)
    hi = np.asarray(color_range.upper, dtype=np.uint8)
    return cv2.inRange(hsv, lo, hi).astype(bool)


ColorRangeSpec = Union[ColorRange, Sequence[ColorRange]]


def _as_ranges(spec: ColorRangeSpec) -> Iterable[ColorRange]:
    if isinstance(spec, ColorRange):
        return (spec,)
    return spec


def make_mask(image: np.ndarray, color_spec: ColorRangeSpec) -> np.ndarray:
    """BGR 画像と ColorRange(単体 or 複数)から bool マスク(H,W)を作る。

    複数の ColorRange を渡すと OR を取る(赤の色相ラップ対応)。
    """
    hsv = bgr_to_hsv(image)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for cr in _as_ranges(color_spec):
        lo = np.asarray(cr.lower, dtype=np.uint8)
        hi = np.asarray(cr.upper, dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    return mask.astype(bool)


def mask_ratio(mask: np.ndarray) -> float:
    """マスクの True 画素割合(0.0-1.0)。解像度非依存のしきい値判定用。"""
    if mask.size == 0:
        return 0.0
    return float(mask.sum()) / float(mask.size)


def clean_mask(
    mask: np.ndarray,
    *,
    kernel_px: int = 7,
    close_iters: int = 2,
    open_iters: int = 1,
) -> np.ndarray:
    """モルフォロジーでマスクのノイズ除去(OPEN)と穴埋め(CLOSE)を行う。

    Twilight の ball_detect.py と同じ楕円カーネル+CLOSE→OPEN。kernel_px<=0
    なら何もしない。入出力とも bool マスク(H,W)。
    """
    if kernel_px <= 0:
        return mask
    m = (mask.astype(np.uint8)) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    if close_iters > 0:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=close_iters)
    if open_iters > 0:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=open_iters)
    return m.astype(bool)
