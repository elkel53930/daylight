"""camera_model.py — 搭載カメラの距離認識モデル(2026-08-04、治具較正)。

下端エッジの row(較正基準 CALIB_HEIGHT=1296 換算)と slope[deg] から、壁までの
距離[mm] と ヨー[deg] を求める。row↔距離・gain↔距離・straight↔距離 がいずれも
非線形(近いほど感度が高い)なので、「90mm付近だけ線形」で割り切らず、治具で
測った較正点を**区分線形補間**して距離依存を反映する(ユーザー方針、2026-08-04)。

較正データ(VGA 768x432・rawをCALIBに固定・crop0.3・下端エッジ・n中央値、治具):
- 距離↔row: 75〜115mm を 5mm刻み(治具で正確に移動)
- gain/straight↔距離: 75 / 90 / 115mm(90mmは −18〜+18°の13点フィット)

⚠️ 限界:
- 近距離限界 ~72〜75mm(それより近いと下端エッジが画像枠外に出て検出不能)。
- 較正範囲は 75〜115mm。範囲外はクランプ(端値)になるので is_in_calib_range で確認する。
- row は yaw で僅かに減少する結合がある(±18°で−60px≈−7mm相当)。実運用のヨーは
  小さい(<5°)ので影響は小さいが、大角度では距離推定に数mmの偏りが出る。
"""
from __future__ import annotations

import bisect
from typing import Tuple

# (distance_mm, row_calib) — yaw≈0 で測定。距離昇順。
_DIST_ROW = [
    (75.0, 1141.4), (80.0, 1005.0), (85.0, 925.6), (90.0, 841.0),
    (95.0, 782.2), (100.0, 736.5), (105.0, 683.9), (110.0, 648.4), (115.0, 607.2),
]
# (distance_mm, gain[slope_deg per yaw_deg], straight_slope_deg) — 距離昇順。
_DIST_GAIN = [
    (75.0, 0.809, 1.477),
    (90.0, 0.608, 0.787),
    (115.0, 0.431, 0.556),
]

CALIB_DIST_MIN = 75.0
CALIB_DIST_MAX = 115.0
CELL_CENTER_MM = 90.0  # マス中心での壁距離

# row は距離とともに単調減少するので、補間用に row 昇順(=距離降順)に並べ替える。
_ROWS_ASC = sorted(_DIST_ROW, key=lambda p: p[1])
_ROW_X = [r for _, r in _ROWS_ASC]
_ROW_D = [d for d, _ in _ROWS_ASC]
_GD_X = [d for d, _, _ in _DIST_GAIN]
_GD_GAIN = [g for _, g, _ in _DIST_GAIN]
_GD_STR = [s for _, _, s in _DIST_GAIN]


def _interp(x: float, xs, ys) -> float:
    """区分線形補間(xs は昇順)。範囲外は端値でクランプ。numpy非依存。"""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x)
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = ys[i - 1], ys[i]
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


def distance_from_row(row_calib: float) -> float:
    """row(較正換算) → 壁までの距離[mm]。較正点の区分線形補間(範囲外はクランプ)。"""
    return _interp(row_calib, _ROW_X, _ROW_D)


def gain_at(dist_mm: float) -> float:
    """距離[mm] における yaw gain(slope_deg / yaw_deg)。"""
    return _interp(dist_mm, _GD_X, _GD_GAIN)


def straight_at(dist_mm: float) -> float:
    """距離[mm] における正対時 slope[deg](straight_slope)。"""
    return _interp(dist_mm, _GD_X, _GD_STR)


_ROW_MIN = min(r for _, r in _DIST_ROW)  # 607.2 (=115mm, 最も遠い)
_ROW_MAX = max(r for _, r in _DIST_ROW)  # 1141.4 (=75mm, 最も近い)


def is_in_calib_range(dist_mm: float) -> bool:
    return CALIB_DIST_MIN <= dist_mm <= CALIB_DIST_MAX


def is_row_in_range(row_calib: float) -> bool:
    """生row が較正範囲内か。distance_from_row はクランプするので、範囲外(近すぎ/
    遠すぎ)判定はクランプ前の row でこちらを使う(row大=近すぎ、row小=遠すぎ)。"""
    return _ROW_MIN <= row_calib <= _ROW_MAX


def estimate(row_calib: float, slope_deg: float) -> Tuple[float, float]:
    """下端エッジの (row_calib, slope[deg]) → (距離[mm], ヨー[deg])。

    ヨー正=機体が左(CCW)を向いている(estimate_pose と同じ規約)。距離は row から、
    ヨーは**その距離の gain/straight** を使って yaw=(slope−straight)/gain で求める。
    """
    dist = distance_from_row(row_calib)
    g = gain_at(dist)
    s0 = straight_at(dist)
    yaw = (slope_deg - s0) / g
    return dist, yaw


def forward_offset_mm(row_calib: float) -> float:
    """マス中心(90mm)からの前後オフセット[mm]。正=中心より前(壁に近い)。"""
    return CELL_CENTER_MM - distance_from_row(row_calib)
