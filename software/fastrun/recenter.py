"""recenter.py — カメラで機体の向きを迷路軸へ再ロックする(2026-08-03〜)。

gyroは旋回のたびに少しずつドリフトする。壁に正対してカメラで赤帯上端エッジ
から機体ヨーを推定し、迷路軸(壁に垂直)へ物理的に向きを揃え直すことで、
探索中のヨー累積ドリフトを消す。ユーザー案「前壁をカメラで確認して位置修正、
90°旋回して側壁でも確認」のうち**ヨー(向き)成分**を実装したもの。位置(並進)
の再ローカライズは距離較正(半セル未取得)+精密微小移動プリミティブ(現FWに
無い)が要るため別途。

前提と安全:
- 正対する壁はマス縁の半セル(≈90mm)距離にある(前壁 or 側壁)。カメラの
  ヨー較正は *_HALFCELL を使う。
- 全ての旋回は hold=True(place_controller の並進+角度保持を切らない)で
  drift を最小化する。RANG/SANG は必ず stop 後に出す(TURN保持中に出すと
  turn_goal と角度フレームがずれ暴走する。CLAUDE.md 参照)。align_to_wall は
  内部で set_reference=True 時に stop→SANG,0 を行う。
- 機体の向きによっては中央クロップに隣の壁が混入して detection が壊れる
  (実測: 東壁=清浄、西壁=CW側で汚染)。align_to_wall は is_confident で
  不良フィットを弾き、その場合は補正せず戻る(安全側で「何もしない」)。
"""
from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np

from camera_align import (
    OnboardCamera,
    PoseEstimate,
    align_to_wall,
    is_confident,
    SLOPE_DEG_PER_YAW_DEG_HALFCELL,
    STRAIGHT_SLOPE_DEG_HALFCELL,
)
from vision_wall import detect_red_band_top_edge

# 搭載カメラの距離較正(2026-08-03、俯瞰を真値にオドメトリ後退で取得):
# 赤帯の row_at_center は壁に1mm近づくごと約5.38px 増加(crop0.3、1px≈0.19mm)。
# 壁上面位置補正の位置成分(row差→mm)に使う。
CAMERA_ROW_PX_PER_MM = 5.38

# 方位規約(超信地旋回・SANG用、deg): 北=0, 西=+90(+CCW=TURN正), 東=-90, 南=±180。
HEADING_NORTH = 0.0
HEADING_WEST = 90.0
HEADING_EAST = -90.0
HEADING_SOUTH = 180.0

# --- カメラによる近距離前壁検出(前壁センサの非単調特性を回避、2026-08-03) ---
# 前壁センサ(lf)は近すぎると値が下がり「近い壁」と「開放」を区別できない
# (ユーザー指摘、CLAUDE.md参照)。代わりにカメラの赤帯上端で判定する。
# 実測(マス中心): 壁@90mm=clean(res<2)6/6・row_at_center≈667、開放=clean0/6
# (res≈50の不良フィット)・row≈241。row_at_center は画面下ほど大=壁が近い。
FRONT_WALL_MAX_RES = 2.0     # これ未満の清浄フィットのみ採用
FRONT_WALL_ROW_MIN = 450     # row_at_center がこれ以上(画面下=近)なら90mm前壁
FRONT_WALL_LF = 300          # (参考)lf しきい値。マス中心限定でしか信頼できない


def front_wall_by_camera(cam: OnboardCamera, *, crop_frac: float = 0.5,
                         n: int = 5) -> bool:
    """カメラで 90mm 前壁の有無を返す(前壁センサの近すぎ低値問題を回避)。

    清浄フィット(res<FRONT_WALL_MAX_RES)かつ row_at_center>FRONT_WALL_ROW_MIN
    (画面下=近い)なフレームが過半数なら壁ありとみなす。row_at_center は
    中央クロップの水平中心での赤帯上端行で、クロップ幅に依らず画像中心の行を指す。
    """
    votes = 0
    for _ in range(n):
        img = cam.capture()
        h, w, _ = img.shape
        half = max(0.05, min(0.5, crop_frac / 2.0))
        lo = int(round(w * (0.5 - half)))
        hi = int(round(w * (0.5 + half)))
        crop = np.ascontiguousarray(img[:, lo:hi, :])
        cw = crop.shape[1]
        e = detect_red_band_top_edge(crop)
        if e is not None and e.residual_std < FRONT_WALL_MAX_RES:
            row_c = e.slope * (cw / 2) + e.intercept
            if row_c > FRONT_WALL_ROW_MIN:
                votes += 1
        time.sleep(0.05)
    return votes >= (n + 1) // 2


def _hold_turn(link, delta_rad: float, *, settle_s: float = 0.8) -> None:
    """相対 delta_rad だけ TURN。stop() は呼ばず保持継続(drift防止)。"""
    if abs(delta_rad) < 1e-4:
        return
    link.send(f"TURN,{delta_rad:.5f}")
    time.sleep(settle_s + abs(delta_rad) / 2.5)


def _read_ang_deg(link):
    for _ in range(8):
        s = link.read_sen()
        if s:
            return math.degrees(s["odo_ang"])
        time.sleep(0.02)
    return None


def turn_to(link, target_deg: float, *, tol: float = 1.0, tries: int = 4) -> Optional[float]:
    """超信地旋回で odo_ang を target_deg[deg] へ最短方向に合わせる(hold継続)。"""
    for _ in range(tries):
        cur = _read_ang_deg(link)
        if cur is None:
            continue
        err = target_deg - cur
        while err > 180.0:
            err -= 360.0
        while err < -180.0:
            err += 360.0
        if abs(err) <= tol:
            return cur
        _hold_turn(link, math.radians(err), settle_s=0.6)
        time.sleep(0.15)
    return _read_ang_deg(link)


def reanchor_heading(link, cam: OnboardCamera, target_heading_deg: float) -> bool:
    """B: カメラで壁に正対してヨーを較正し、heading を絶対方位へ貼り直す。

    target_heading_deg は「その壁に正対したときの真の方位」(北=0/西=+90/東=-90/
    南=±180)。既知マスの既知の壁を選んで呼ぶ。手順:
      1. その方位へ超信地旋回で大まかに向く(gyroはドリフトしていてOK)
      2. align_to_wall(半セル, crop0.3)で壁に正確に正対(角度成分の壁上面位置補正)
      3. is_confident でなければ諦める(汚染/壁掴めず)
      4. stop→SANG で odo_ang を target_heading_deg に上書き(=絶対基準に貼り直す)
    停止中はジャイロ積分が凍結される(FW側)ので、この基準は静止中ドリフトしない。
    成功で True。
    """
    turn_to(link, target_heading_deg)
    time.sleep(0.2)
    est = align_to_wall(
        link, cam, iterations=5, deadband_deg=0.8, max_step_deg=15,
        set_reference=False, stop_at_end=True, check_dist=False,
        slope_gain=SLOPE_DEG_PER_YAW_DEG_HALFCELL,
        straight_slope=STRAIGHT_SLOPE_DEG_HALFCELL, crop_frac=0.3,
    )
    if est is None or not is_confident(est, check_dist=False):
        return False
    link.stop()
    time.sleep(0.2)
    link.send(f"SANG,{math.radians(target_heading_deg):.5f}")
    link.wait_for("DONE", timeout_s=1.0)
    return True


def relock_heading(
    link,
    cam: OnboardCamera,
    face_steps: int,
    *,
    iterations: int = 5,
    deadband_deg: float = 0.8,
) -> Optional[PoseEstimate]:
    """face_steps だけ回って壁に正対→カメラでヨー補正→元の向きへ戻す。

    face_steps: +1=右90°(CW), -1=左90°(CCW), ±2=180°。0 なら今の前壁で補正。
    戻り値は align_to_wall の推定(ログ用、補正できなければ None 相当)。

    手順:
      1. face_steps ぶん hold 旋回して壁に正対
      2. align_to_wall(半セル較正, set_reference=True): ヨーを詰めて
         内部で stop→SANG,0(=正対を角度0基準に)
      3. -face_steps ぶん hold 旋回して元の向きへ戻す
      4. stop→RANG で gyro を「元の向き=迷路軸」に0基準化
    正味: gyro を壁基準で再ロックし、機体を迷路軸へ物理的に揃え直す。
    """
    turn = -face_steps * (math.pi / 2.0)  # +1(右/CW) = 負のrad
    _hold_turn(link, turn)
    time.sleep(0.2)

    est = align_to_wall(
        link, cam,
        iterations=iterations, deadband_deg=deadband_deg, max_step_deg=15,
        set_reference=True, check_dist=False,
        slope_gain=SLOPE_DEG_PER_YAW_DEG_HALFCELL,
        straight_slope=STRAIGHT_SLOPE_DEG_HALFCELL,
        crop_frac=0.3,  # 側壁正対時の隣壁混入を減らす(清浄な角度範囲を広げる)
    )
    # align_to_wall は set_reference=True 時に内部で stop→SANG,0 済み。
    # ここから元の向きへ戻す(逆回転)。
    _hold_turn(link, -turn)
    link.stop()
    time.sleep(0.2)
    link.send("RANG")
    link.wait_for("DONE", timeout_s=2.0)
    return est


def choose_face_steps(front: bool, left: bool, right: bool) -> Optional[int]:
    """再ロックに使う壁を選び face_steps を返す。壁が無ければ None。

    前壁があれば旋回不要(0)。無ければ側壁(右優先=CW側は汚染しやすいので
    実際は呼び出し側が向きに応じて選ぶ余地あり。ここでは単純に前>右>左)。
    """
    if front:
        return 0
    if right:
        return 1   # 右90°で右壁を正面に
    if left:
        return -1  # 左90°で左壁を正面に
    return None
