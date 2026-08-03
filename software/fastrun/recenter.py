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

from camera_align import (
    OnboardCamera,
    PoseEstimate,
    align_to_wall,
    SLOPE_DEG_PER_YAW_DEG_HALFCELL,
    STRAIGHT_SLOPE_DEG_HALFCELL,
)

# 前壁センサ(lf)がこの値を超えたら 90mm 直前に壁ありとみなす
# (実測: 壁@90mm≈800、開放≈2。中間の 1.5セル先壁≈188 は跨がない保守しきい値)。
FRONT_WALL_LF = 300


def _hold_turn(link, delta_rad: float, *, settle_s: float = 0.8) -> None:
    """相対 delta_rad だけ TURN。stop() は呼ばず保持継続(drift防止)。"""
    if abs(delta_rad) < 1e-4:
        return
    link.send(f"TURN,{delta_rad:.5f}")
    time.sleep(settle_s + abs(delta_rad) / 2.5)


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
