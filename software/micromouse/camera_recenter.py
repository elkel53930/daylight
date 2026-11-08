"""カメラによる位置補正.

機体の位置・姿勢をカメラで補正する。壁のある方向に旋回し、赤エッジを検出して
ヨー角と前後位置を補正する。

機能:
- recenter_axis: 指定方向の壁に対して1軸を補正
- recenter_cell: X/Y/XY軸の補正を実行

手順:
1. 補正対象の壁の方向を決定(X方向=Y軸の壁、Y方向=X軸の壁)
2. その方向に旋回
3. カメラで赤エッジ検出 → ヨー角・前後位置を推定
4. TURN/FWD/STOP で補正
5. 元の向きに戻る
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "fastrun"))

from camera_align import OnboardCamera, CALIB_HEIGHT
from camera_model import estimate, forward_offset_mm, is_row_in_range
from wall_bottom import detect_red_band_bottom_edge

CROP_FRAC = 0.3
MAX_RESIDUAL_PX = 2.0
YAW_TOL_DEG = 1.0
OFFSET_TOL_MM = 3.0
MAX_YAW_CORRECTION_DEG = 15.0
MAX_OFFSET_CORRECTION_MM = 30.0
N_CAPTURES = 5


class Axis(Enum):
    X = "x"
    Y = "y"
    XY = "xy"


class Direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def turn_rad_to(self, other: "Direction") -> float:
        """この方向から other への旋回角[rad]を返す(正=左/CCW)."""
        diff = (other.value - self.value) % 4
        if diff == 0:
            return 0.0
        elif diff == 1:
            return -math.pi / 2
        elif diff == 2:
            return math.pi
        else:
            return math.pi / 2


@dataclass(frozen=True)
class RecenterResult:
    axis: str
    yaw_deg: float
    offset_mm: float
    success: bool
    message: str


def _capture_and_estimate(cam: OnboardCamera) -> Optional[Tuple[float, float, float]]:
    rows = []
    slopes = []
    ress = []
    for _ in range(N_CAPTURES):
        img = cam.capture()
        h, w, _ = img.shape
        half = max(0.05, min(0.5, CROP_FRAC / 2.0))
        lo = int(round(w * (0.5 - half)))
        hi = int(round(w * (0.5 + half)))
        crop = np.ascontiguousarray(img[:, lo:hi, :])
        e = detect_red_band_bottom_edge(crop)
        if e is not None and e.residual_std < MAX_RESIDUAL_PX:
            cw = crop.shape[1]
            row_calib = (e.slope * (cw / 2) + e.intercept) * (CALIB_HEIGHT / h)
            slope_deg = math.degrees(math.atan(e.slope))
            rows.append(row_calib)
            slopes.append(slope_deg)
            ress.append(e.residual_std)
        time.sleep(0.04)
    if not rows:
        return None
    row_med = float(np.median(rows))
    slope_med = float(np.median(slopes))
    res_med = float(np.median(ress))
    return row_med, slope_med, res_med


def recenter_axis(
    base,
    cam: OnboardCamera,
    current_heading: Direction,
    wall_direction: Direction,
    *,
    log_fn=None,
) -> RecenterResult:
    """指定方向の壁に対して1軸を補正する.

    Args:
        base: MobileBase
        cam: OnboardCamera
        current_heading: 現在の機体の向き
        wall_direction: 壁がある方向(この方向の壁に対して補正)
        log_fn: ログ関数

    Returns:
        RecenterResult
    """
    def log(**record):
        if log_fn:
            log_fn(event="recenter_axis", **record)

    wall_direction_name = wall_direction.name.lower()
    log(message=f"start axis={wall_direction_name}")

    # 壁の方向に旋回
    turn_rad = current_heading.turn_rad_to(wall_direction)
    if abs(turn_rad) > 0.01:
        base.turn(turn_rad)
        log(message=f"turned {turn_rad:+.3f}rad to face {wall_direction_name}")
        time.sleep(0.3)

    # カメラで補正
    result = _capture_and_estimate(cam)
    if result is None:
        log(message="no wall detected")
        # 元の向きに戻る
        if abs(turn_rad) > 0.01:
            base.turn(-turn_rad)
        return RecenterResult(
            axis=wall_direction_name,
            yaw_deg=0,
            offset_mm=0,
            success=False,
            message="no wall detected",
        )

    row_med, slope_med, res_med = result
    dist, yaw_deg = estimate(row_med, slope_med)
    offset_mm = forward_offset_mm(row_med)
    log(message=f"measured: dist={dist:.0f}mm yaw={yaw_deg:+.1f}deg offset={offset_mm:+.1f}mm")

    # ヨー補正
    if abs(yaw_deg) > YAW_TOL_DEG and abs(yaw_deg) <= MAX_YAW_CORRECTION_DEG:
        correction_rad = -math.radians(yaw_deg)
        base.turn(correction_rad)
        log(message=f"yaw corrected: {-yaw_deg:+.1f}deg")
        time.sleep(0.2)
        # 再測定
        result2 = _capture_and_estimate(cam)
        if result2 is not None:
            row_med, _, _ = result2
            offset_mm = forward_offset_mm(row_med)

    # 前後補正
    if abs(offset_mm) > OFFSET_TOL_MM and abs(offset_mm) <= MAX_OFFSET_CORRECTION_MM:
        move = -offset_mm
        speed = getattr(base, "config", None)
        speed_mmps = speed.explore_speed_mmps if speed else 100.0
        accel_mmps2 = speed.explore_accel_mmps2 if speed else 500.0
        if move > 0:
            base.forward(speed_mmps, accel_mmps2, move)
        else:
            base.stop_at(speed_mmps, accel_mmps2, -move)
        log(message=f"offset corrected: move {move:+.1f}mm")

    # 元の向きに戻る
    if abs(turn_rad) > 0.01:
        base.turn(-turn_rad)
        log(message=f"turned back {-turn_rad:+.3f}rad")

    log(message=f"axis {wall_direction_name} done")
    return RecenterResult(
        axis=wall_direction_name,
        yaw_deg=yaw_deg,
        offset_mm=offset_mm,
        success=True,
        message=f"yaw={yaw_deg:+.1f}deg offset={offset_mm:+.1f}mm",
    )


def recenter_cell(
    base,
    cam: OnboardCamera,
    current_heading: Direction,
    walls: dict,
    axis: Axis = Axis.XY,
    *,
    log_fn=None,
) -> list[RecenterResult]:
    """セルの位置補正を実行する(fastrun/liner_center.py の center_axis と同等).

    2測定・2一発補正(オープンループ):
      (1) 壁の方向に旋回
      (2) 【測定1】下端エッジで角度(ヨー)を測り、-yaw だけ TURN で一発旋回
      (3) SANG で絶対方位を確定
      (4) 【測定2】下端エッジで前後距離オフセットを測り、JOGFWD/JOGBACK で移動
      (5) 次の壁の方向へ連続旋回

    Args:
        base: MobileBase
        cam: OnboardCamera
        current_heading: 現在の機体の向き
        walls: 壁情報 {"north": bool, "east": bool, "south": bool, "west": bool}
        axis: 補正する軸(X=x/y方向の壁、Y=y方向の壁、XY=両方)
        log_fn: ログ関数

    Returns:
        RecenterResult のリスト
    """
    results = []
    total_turn_rad = 0.0  # 累積旋回角

    # 補正対象の壁の方向を決定
    target_directions = []

    if axis in (Axis.X, Axis.XY):
        for wall_dir in [Direction.EAST, Direction.WEST]:
            wall_name = wall_dir.name.lower()
            if walls.get(wall_name, False):
                target_directions.append(wall_dir)
                break

    if axis in (Axis.Y, Axis.XY):
        for wall_dir in [Direction.NORTH, Direction.SOUTH]:
            wall_name = wall_dir.name.lower()
            if walls.get(wall_name, False):
                target_directions.append(wall_dir)
                break

    if not target_directions:
        return results

    # 各壁に対して連続旋回で補正
    for wall_dir in target_directions:
        wall_name = wall_dir.name.lower()

        # 現在の向きから壁の方向への旋回角を計算
        turn_rad = current_heading.turn_rad_to(wall_dir)
        actual_turn_rad = turn_rad - total_turn_rad

        # 旋回
        if abs(actual_turn_rad) > 0.01:
            base.turn(actual_turn_rad)
            log_fn and log_fn(event="recenter_cell", message=f"turned {actual_turn_rad:+.3f}rad to face {wall_name}")
            time.sleep(0.3)

        total_turn_rad = turn_rad

        # 【測定1】ヨーを測定
        result1 = _capture_and_estimate(cam)
        if result1 is None:
            log_fn and log_fn(event="recenter_cell", message=f"no wall detected at {wall_name} (measure1)")
            results.append(RecenterResult(
                axis=wall_name,
                yaw_deg=0,
                offset_mm=0,
                success=False,
                message="no wall detected (measure1)",
            ))
            continue

        row_med1, slope_med1, res_med1 = result1
        dist1, yaw_deg1 = estimate(row_med1, slope_med1)
        offset_mm1 = forward_offset_mm(row_med1)
        log_fn and log_fn(event="recenter_cell", message=f"measure1: dist={dist1:.0f}mm yaw={yaw_deg1:+.1f}deg at {wall_name}")

        # ヨー補正（範囲内の場合）
        yaw_corrected = False
        if abs(yaw_deg1) > YAW_TOL_DEG and abs(yaw_deg1) <= MAX_YAW_CORRECTION_DEG:
            correction_rad = -math.radians(yaw_deg1)
            base.turn(correction_rad)
            total_turn_rad += correction_rad
            log_fn and log_fn(event="recenter_cell", message=f"yaw corrected: {-yaw_deg1:+.1f}deg")
            time.sleep(0.3)
            yaw_corrected = True

            # SANG で絶対方位を確定（現在の向きを目標方位に設定）
            # target_angle = 壁の方向の絶対角度
            target_angle_rad = {
                Direction.NORTH: 0.0,
                Direction.EAST: -math.pi / 2,
                Direction.SOUTH: math.pi,
                Direction.WEST: math.pi / 2,
            }[wall_dir]
            base.set_angle(target_angle_rad)
            log_fn and log_fn(event="recenter_cell", message=f"SANG set to {math.degrees(target_angle_rad):+.1f}deg")
            time.sleep(0.1)

        # 【測定2】正対後に距離を測定
        result2 = _capture_and_estimate(cam)
        if result2 is None:
            log_fn and log_fn(event="recenter_cell", message=f"no wall detected at {wall_name} (measure2)")
            results.append(RecenterResult(
                axis=wall_name,
                yaw_deg=yaw_deg1,
                offset_mm=offset_mm1,
                success=yaw_corrected,
                message="no wall detected (measure2)",
            ))
            continue

        row_med2, slope_med2, res_med2 = result2
        dist2, yaw_deg2 = estimate(row_med2, slope_med2)
        offset_mm2 = forward_offset_mm(row_med2)
        log_fn and log_fn(event="recenter_cell", message=f"measure2: dist={dist2:.0f}mm yaw={yaw_deg2:+.1f}deg offset={offset_mm2:+.1f}mm at {wall_name}")

        # 前後補正
        if abs(offset_mm2) > OFFSET_TOL_MM and abs(offset_mm2) <= MAX_OFFSET_CORRECTION_MM:
            move = -offset_mm2
            speed = getattr(base, "config", None)
            speed_mmps = speed.explore_speed_mmps if speed else 100.0
            accel_mmps2 = speed.explore_accel_mmps2 if speed else 500.0
            if move > 0:
                base.forward(speed_mmps, accel_mmps2, move)
            else:
                base.stop_at(speed_mmps, accel_mmps2, -move)
            log_fn and log_fn(event="recenter_cell", message=f"offset corrected: move {move:+.1f}mm")

        results.append(RecenterResult(
            axis=wall_name,
            yaw_deg=yaw_deg1,
            offset_mm=offset_mm2,
            success=True,
            message=f"yaw={yaw_deg1:+.1f}deg offset={offset_mm2:+.1f}mm",
        ))

    # 最後に元の向きに戻る
    return_turn_rad = -total_turn_rad
    if abs(return_turn_rad) > 0.01:
        base.turn(return_turn_rad)
        log_fn and log_fn(event="recenter_cell", message=f"turned back {return_turn_rad:+.3f}rad to original heading")

    return results


def recenter_start(base, current_heading: Direction, walls: dict, *, log_fn=None) -> list[RecenterResult]:
    """スタート位置の補正を実行する(XY両軸)."""
    cam = None
    try:
        cam = OnboardCamera()
        results = recenter_cell(base, cam, current_heading, walls, Axis.XY, log_fn=log_fn)
        return results
    except Exception as e:
        return [RecenterResult("xy", 0, 0, False, f"camera init failed: {e}")]
    finally:
        if cam:
            try:
                cam.close()
            except Exception:
                pass
