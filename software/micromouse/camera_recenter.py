"""カメラによる位置補正(fastrun/Eiffel経路走行と同等方式).

マイクロマウスのスタート時などに、壁上面の赤エッジをカメラで検出し、
ヨー(向き)と前後位置をマス中心へ補正する。

Eiffel経路走行(fastrun/liner_center.center_axis)に合わせた方式:
- 対象壁にジャイロ絶対角で正対
- レンジゲート: 壁が較正範囲(75〜115mm)外なら JOG で寄せて再測定
- ヨー: 測定→TURN を反復して物理的に正対(deadband 0.8°)→ MOT,0,0 → SANG
- 位置: JOGFWD/JOGBACK で精密移動(低速・到達停止)、再測定しながら収束
- クリーンフレーム過半数を要求(単発の誤検出を排除)
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

_FASTRUN_DIR = str(Path(__file__).resolve().parent.parent / "fastrun")
if _FASTRUN_DIR not in sys.path:
    sys.path.append(_FASTRUN_DIR)  # 先頭に入れるとmicromouseのexplorer/mazeを隠すため末尾

from camera_align import OnboardCamera, CALIB_HEIGHT
from camera_model import (
    estimate,
    forward_offset_mm,
    is_row_in_range,
    row_range_state,
)
from wall_bottom import detect_red_band_bottom_edge

try:
    from errors import AbortRequested
except ImportError:  # pragma: no cover
    class AbortRequested(Exception):
        pass


def _check_abort(base) -> None:
    """走行中のユーザー中断(Lボタン等)を検出したらモーター停止して送出."""
    ac = getattr(base, "abort_check", None)
    if ac is not None and ac():
        try:
            base.emergency_stop()
        finally:
            raise AbortRequested("aborted by user")

CROP_FRAC = 0.3
MAX_RESIDUAL_PX = 2.0
N_CAPTURES = 5

YAW_DEADBAND_DEG = 0.8
YAW_GATE_DEG = 15.0
YAW_MAX_ITER = 5

RANGE_ADJUST_MM = 15.0
MAX_RANGE_STEPS = 4
CENTER_TOL_MM = 4.0
POS_MAX_ITER = 3
MAX_OFFSET_MM = 45.0


class Axis(Enum):
    X = "x"
    Y = "y"
    XY = "xy"


class Direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


@dataclass(frozen=True)
class Measurement:
    row_calib: float
    slope_deg: float
    res_px: float
    n_clean: int
    in_range: bool
    ok: bool
    dist_mm: float
    yaw_deg: float
    offset_mm: float


@dataclass(frozen=True)
class RecenterResult:
    axis: str
    yaw_deg: float
    offset_mm: float
    success: bool
    message: str


def _wall_angle_rad(d: Direction) -> float:
    """壁に正対したときの絶対方位[rad] (北=0, 西=+90, 東=-90, 南=180)."""
    return {
        Direction.NORTH: 0.0,
        Direction.WEST: math.pi / 2,
        Direction.EAST: -math.pi / 2,
        Direction.SOUTH: math.pi,
    }[d]


def _norm_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _measure(cam: OnboardCamera) -> Optional[Measurement]:
    """複数フレームを撮り、クリーンなフレームの中央値で (距離, ヨー, offset) を返す."""
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
    dist, yaw = estimate(row_med, slope_med)
    offset = forward_offset_mm(row_med)
    in_range = is_row_in_range(row_med)
    majority = len(rows) >= (N_CAPTURES + 1) // 2
    ok = majority and in_range
    return Measurement(
        row_calib=row_med,
        slope_deg=slope_med,
        res_px=res_med,
        n_clean=len(rows),
        in_range=in_range,
        ok=ok,
        dist_mm=dist,
        yaw_deg=yaw,
        offset_mm=offset,
    )


def _measure_in_range(base, cam: OnboardCamera, log) -> Optional[Measurement]:
    """壁が較正範囲に入るまで JOG で寄せて再測定する(Eiffel方式)."""
    for attempt in range(MAX_RANGE_STEPS + 1):
        _check_abort(base)
        m = _measure(cam)
        if m is None:
            log(event="measure", stage=f"attempt{attempt}", result="none")
            return None
        log(
            event="measure",
            stage=f"attempt{attempt}",
            in_range=m.in_range,
            ok=m.ok,
            n_clean=m.n_clean,
            dist=round(m.dist_mm, 1),
            yaw=round(m.yaw_deg, 2),
            offset=round(m.offset_mm, 1),
            row=round(m.row_calib, 1),
        )
        if m.ok or attempt >= MAX_RANGE_STEPS:
            return m
        state = row_range_state(m.row_calib)
        if state == "far":
            base.jog_forward(RANGE_ADJUST_MM)
        elif state == "near":
            base.jog_backward(RANGE_ADJUST_MM)
        else:
            # 範囲内だがクリーン不足(汚染): それ以上寄せず返す
            return m
        time.sleep(0.2)
    return None


def _turn_to_abs(base, target_rad: float, log, tol_deg: float = 1.0, tries: int = 4) -> Optional[float]:
    """ジャイロ絶対角で target_rad へ超信地旋回(Eiffelのturn_to相当)."""
    cur = None
    for _ in range(tries):
        cur = base.read_angle()
        if cur is None:
            return None
        err = _norm_rad(target_rad - cur)
        if abs(math.degrees(err)) <= tol_deg:
            return cur
        base.turn(err)
        time.sleep(0.15)
    return base.read_angle()


def _align_yaw(base, cam: OnboardCamera, target_rad: float, log) -> Tuple[bool, Optional[Measurement]]:
    """測定→TURN を反復して物理的に壁へ正対し、停止後に SANG で絶対方位を貼る."""
    last: Optional[Measurement] = None
    for _ in range(YAW_MAX_ITER):
        _check_abort(base)
        m = _measure_in_range(base, cam, log)
        if m is None:
            return False, last
        last = m
        if abs(m.yaw_deg) <= YAW_DEADBAND_DEG:
            break
        if abs(m.yaw_deg) > YAW_GATE_DEG:
            log(event="yaw", message=f"too large {m.yaw_deg:+.1f}deg -> skip")
            return False, m
        base.turn(-math.radians(m.yaw_deg))
        time.sleep(0.2)

    # 停止してからSANG(ジャイロ角のみ書き換えるので物理整合は上で済ませる)
    base.motors_off()
    time.sleep(0.1)
    base.set_angle(target_rad)
    time.sleep(0.1)
    m2 = _measure_in_range(base, cam, log)
    return True, (m2 or last)


def _center_position(base, cam: OnboardCamera, log) -> Tuple[bool, Optional[Measurement]]:
    """JOGで前後位置をマス中心(壁90mm)へ寄せる."""
    last: Optional[Measurement] = None
    for _ in range(POS_MAX_ITER):
        _check_abort(base)
        m = _measure_in_range(base, cam, log)
        if m is None:
            return False, last
        last = m
        off = m.offset_mm
        if abs(off) <= CENTER_TOL_MM:
            return True, m
        if abs(off) > MAX_OFFSET_MM:
            log(event="offset", message=f"too large {off:+.1f}mm -> skip")
            return False, m
        if off > 0:
            # 中心より前(壁に近い) -> 後退
            base.jog_backward(off)
        else:
            base.jog_forward(-off)
        time.sleep(0.15)
    m = _measure_in_range(base, cam, log)
    return (m is not None), (m or last)


def recenter_cell(
    base,
    cam: OnboardCamera,
    current_heading: Direction,
    walls: dict,
    axis: Axis = Axis.XY,
    *,
    log_fn=None,
) -> list[RecenterResult]:
    """セルの位置補正(X/Y/XY)を実行する.

    walls: {"north":bool,"east":bool,"south":bool,"west":bool}
    """
    def log(**record):
        if log_fn:
            record.setdefault("event", "recenter_cell")
            log_fn(**record)

    results: list[RecenterResult] = []
    heading0 = base.read_angle()  # 元の向き(補正後に戻る)

    target_directions: list[Direction] = []
    if axis in (Axis.X, Axis.XY):
        for d in [Direction.EAST, Direction.WEST]:
            if walls.get(d.name.lower(), False):
                target_directions.append(d)
                break
    if axis in (Axis.Y, Axis.XY):
        for d in [Direction.NORTH, Direction.SOUTH]:
            if walls.get(d.name.lower(), False):
                target_directions.append(d)
                break

    for d in target_directions:
        name = d.name.lower()
        target_rad = _wall_angle_rad(d)
        log(event="face", wall=name, target_deg=round(math.degrees(target_rad), 1))
        _turn_to_abs(base, target_rad, log)
        base.motors_off()
        time.sleep(0.1)

        ok_yaw, m_yaw = _align_yaw(base, cam, target_rad, log)
        ok_pos, m_pos = _center_position(base, cam, log)

        yaw = m_yaw.yaw_deg if m_yaw is not None else 0.0
        offset = m_pos.offset_mm if m_pos is not None else 0.0
        success = bool(ok_yaw and ok_pos)
        results.append(
            RecenterResult(
                axis=name,
                yaw_deg=yaw,
                offset_mm=offset,
                success=success,
                message=f"yaw={yaw:+.1f}deg offset={offset:+.1f}mm",
            )
        )
        log(event="axis_done", wall=name, success=success)

    # 元の向き(北)へ戻す
    if heading0 is not None:
        _turn_to_abs(base, heading0, log)
        base.motors_off()

    return results


def recenter_start(base, current_heading: Direction, walls: dict, *, log_fn=None) -> list[RecenterResult]:
    """スタート位置の補正を実行する(XY両軸)."""
    cam = None
    try:
        cam = OnboardCamera()
        return recenter_cell(base, cam, current_heading, walls, Axis.XY, log_fn=log_fn)
    except Exception as e:
        return [RecenterResult("xy", 0, 0, False, f"camera init failed: {e}")]
    finally:
        if cam:
            try:
                cam.close()
            except Exception:
                pass
