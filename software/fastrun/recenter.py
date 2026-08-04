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
from dataclasses import dataclass
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
# ⚠️ この線形較正は90mm付近でしか有効でない(2026-08-04実走で判明)。row→距離は
# 遠近法で非線形: 遠距離ほどrowが距離に鈍感(実測 遠距離≈0.97px/mm→近距離≈7.8px/mm、
# 約8倍差)。遠い壁(row小)に線形外挿すると距離を大幅に過小評価する(実288mmを183mmと
# 誤認した)。→ 遠距離からの接近は row 自体(距離に単調)を信号にし、近距離域(row≈690)
# でだけこのmm較正を使う(center_on_front_wall がそれを行う)。
CAMERA_ROW_PX_PER_MM = 5.38
# 壁が90mm(マス中心)のときの row_at_center(crop0.3)。専用治具で(1,1)中心・西向きに
# 正確固定して取得(2026-08-03、res<=0.38, n=10の精密値)。壁上面位置補正の位置成分の
# 絶対基準: 現在のrowとの差 (row-CAMERA_ROW_AT_90MM)/CAMERA_ROW_PX_PER_MM [mm] で
# 壁までの距離のマス中心(90mm)からのずれが出る。
CAMERA_ROW_AT_90MM = 689.9

# 位置成分(壁上面位置補正)の信頼度ゲート。
FORWARD_OFFSET_MAX_RES = 2.0   # 赤帯フィット残差[px]の上限(角度成分と同じ清浄判定)
FORWARD_OFFSET_MAX_MM = 45.0   # 単発で許す最大前後オフセット[mm](較正範囲±30mmに
                               # マージン。これを超える推定は較正外/誤検出の疑いが
                               # 強く、confident=False にして移動に使わせない)


@dataclass(frozen=True)
class ForwardOffset:
    """前後位置の推定(壁上面位置補正の位置成分)。

    offset_mm > 0: 機体がマス中心(壁90mm)より前=壁に近い側にいる。中心へ戻すには
    offset_mm だけ後退する。< 0 は前進で中心へ。
    """
    offset_mm: float
    row_at_center: float
    residual_px: float
    confident: bool


def forward_offset_from_row(row_at_center: float) -> float:
    """赤帯 row_at_center[px] から前後オフセット[mm]を返す(治具検証済み定数)。

    row は画面下ほど大=壁が近い。90mm(マス中心)基準の row=CAMERA_ROW_AT_90MM から
    1mm近づくごと CAMERA_ROW_PX_PER_MM 増える。正=中心より前(壁に近い)。純関数。
    """
    return (row_at_center - CAMERA_ROW_AT_90MM) / CAMERA_ROW_PX_PER_MM


def forward_offset_from_image(img: np.ndarray, *, crop_frac: float = 0.3) -> Optional[ForwardOffset]:
    """1枚のRGB画像から前後オフセットを推定する純関数(ハード非依存、テスト可能)。

    中央 crop_frac 幅クロップ→赤帯上端エッジ→水平中心での row_at_center→mm換算。
    赤帯を検出できなければ None。confident は残差とオフセット範囲のゲート。
    crop_frac は治具較正(CAMERA_ROW_AT_90MM)と同じ 0.3 を既定にすること
    (row_at_center は水平中心の行なのでクロップ幅にほぼ不変だが、較正と揃える)。
    """
    h, w, _ = img.shape
    half = max(0.05, min(0.5, crop_frac / 2.0))
    lo = int(round(w * (0.5 - half)))
    hi = int(round(w * (0.5 + half)))
    crop = np.ascontiguousarray(img[:, lo:hi, :])
    e = detect_red_band_top_edge(crop)
    if e is None:
        return None
    cw = crop.shape[1]
    row_c = e.slope * (cw / 2) + e.intercept
    offset = forward_offset_from_row(row_c)
    confident = (e.residual_std < FORWARD_OFFSET_MAX_RES
                 and abs(offset) <= FORWARD_OFFSET_MAX_MM)
    return ForwardOffset(offset_mm=offset, row_at_center=row_c,
                         residual_px=e.residual_std, confident=confident)


# --- 前壁への接近+中心化(壁上面位置補正の位置成分、2026-08-04実走検証) ---
# row_at_center→距離[mm]の線形較正(CAMERA_ROW_PX_PER_MM)は90mm付近でしか
# 有効でない(遠近法で遠距離ほどrowが距離に鈍感、実測 遠0.97px/mm→近7.8px/mm)。
# よって接近は「row(距離に単調)」を信号に近距離域(row≈690=90mm)へ寄せ、近距離
# だけmm較正で微調整する。衝突ガードもrow(信頼できる)で行う。
ROW_NEAR_STOP = 650.0    # rowがこれ以上で近距離域(≈95mm)に到達→接近停止
ROW_TOO_CLOSE = 720.0    # rowがこれ超で近すぎ(≈<85mm)→後退で戻す
APPROACH_TOTAL_CAP_MM = 240.0  # 接近の総前進上限(暴走バックストップ)
CENTER_TOL_MM = 4.0      # |offset|がこれ以下で中心とみなす
# 到達時に壁へ寄りすぎ(高速移動のオーバーシュート等)て赤エッジが枠外/未検出の
# とき、既知の壁がある前提なら少し後退して視野に入れてから中心化する。
BACKOFF_STEP_MM = 25.0
BACKOFF_CAP_MM = 90.0


def approach_step_mm(row: float) -> float:
    """rowに応じた接近ステップ[mm](純関数)。遠い(row小)ほど大きく、近い
    (row大=急変域)ほど小さく刻んで、近距離での行き過ぎを防ぐ。"""
    if row < 350.0:
        return 30.0
    if row < 500.0:
        return 18.0
    if row < 600.0:
        return 10.0
    return 6.0


def measure_front_row(cam: OnboardCamera, *, crop_frac: float = 0.3,
                      n: int = 5) -> tuple[Optional[float], Optional[float]]:
    """前壁赤帯の row_at_center と残差の中央値(検出フレームのみ)。撮影のみ。"""
    rows: list[float] = []
    ress: list[float] = []
    for _ in range(n):
        est = forward_offset_from_image(cam.capture(), crop_frac=crop_frac)
        if est is not None:
            rows.append(est.row_at_center)
            ress.append(est.residual_px)
        time.sleep(0.04)
    if not rows:
        return None, None
    return float(np.median(rows)), float(np.median(ress))


def _jog(link, cmd: str, *, timeout_s: float = 9.0) -> bool:
    """JOG系コマンドを送り DONE を待つ(JOGだけは到達でDONEを返す)。"""
    link.send(cmd)
    return link.wait_for("DONE", timeout_s=timeout_s) is not None


def center_on_front_wall(link, cam: OnboardCamera, *,
                         res_max: float = FORWARD_OFFSET_MAX_RES,
                         crop_frac: float = 0.3,
                         expect_wall: bool = True) -> Optional[ForwardOffset]:
    """前壁に対しマス中心(壁90mm)へ寄せる(壁上面位置補正の位置成分)。

    2段階: (1)row駆動の接近でrowをROW_NEAR_STOP(≈95mm)まで前進、(2)近距離域で
    mm較正を使い JOGFWD/JOGBACK で offset→0 に微調整。前壁が近距離域より遠くても
    近距離まで安全に寄せられる。汚染(res>res_max)・検出不能・総前進上限で安全中断。
    ⚠️ 呼ぶ前に前壁へ概ね正対していること(必要なら reanchor_heading/relock で先に
    正対)。低速JOGはヨーを保持するので接近中に横へ逸れにくいが、初期のヨーずれは
    そのまま横位置ずれになる。戻り値は最終の前後オフセット推定(Noneは失敗)。
    """
    # (1) row駆動の接近
    total_fwd = 0.0
    back_off_total = 0.0
    for _ in range(40):
        row, res = measure_front_row(cam, crop_frac=crop_frac, n=5)
        if row is None or res is None or res > res_max:
            # 未検出/汚染。既知の壁がある前提(expect_wall)なら、壁へ寄りすぎて枠外に
            # なっている可能性があるので少し後退して視野に入れてから再測定する。
            # 上限まで後退しても駄目なら本当に壁が無い/掴めないとして中断。
            if expect_wall and back_off_total < BACKOFF_CAP_MM:
                _jog(link, f"JOGBACK,{BACKOFF_STEP_MM:.1f}")
                back_off_total += BACKOFF_STEP_MM
                time.sleep(0.15)
                continue
            return None
        if row > ROW_TOO_CLOSE:
            back = (row - CAMERA_ROW_AT_90MM) / CAMERA_ROW_PX_PER_MM
            _jog(link, f"JOGBACK,{max(2.0, back):.1f}")
            continue
        if row >= ROW_NEAR_STOP:
            break  # 近距離域に到達
        step = approach_step_mm(row)
        if total_fwd + step > APPROACH_TOTAL_CAP_MM:
            break  # 総前進上限(バックストップ)
        total_fwd += step
        _jog(link, f"JOGFWD,{step:.1f}")
        time.sleep(0.2)

    # (2) 近距離域での mm 微調整
    last: Optional[ForwardOffset] = None
    for _ in range(6):
        row, res = measure_front_row(cam, crop_frac=crop_frac, n=6)
        if row is None or res is None or res > res_max:
            return last
        off = forward_offset_from_row(row)
        last = ForwardOffset(offset_mm=off, row_at_center=row,
                             residual_px=res, confident=abs(off) <= FORWARD_OFFSET_MAX_MM)
        if abs(off) <= CENTER_TOL_MM:
            return last
        move = max(-15.0, min(15.0, -off))  # +前進/-後退、1回15mm上限
        if move > 0:
            _jog(link, f"JOGFWD,{move:.1f}")
        else:
            _jog(link, f"JOGBACK,{-move:.1f}")
        time.sleep(0.2)
    return last


def estimate_forward_offset(cam: OnboardCamera, *, crop_frac: float = 0.3,
                            n: int = 5) -> Optional[ForwardOffset]:
    """搭載カメラで前後オフセットを推定(撮影のみ=移動しない=安全)。

    n フレーム撮り、検出できた row_at_center の中央値で頑健化する。清浄フィット
    (res<FORWARD_OFFSET_MAX_RES)が過半数に満たない、または offset が範囲外なら
    confident=False(=中心化の移動に使ってはいけない)。実際の前後移動は呼び出し側
    (監督下)が行う。ここは絶対基準の測定のみを担う。
    """
    rows: list[float] = []
    ress: list[float] = []
    for _ in range(n):
        est = forward_offset_from_image(cam.capture(), crop_frac=crop_frac)
        if est is not None:
            rows.append(est.row_at_center)
            ress.append(est.residual_px)
        time.sleep(0.05)
    if not rows:
        return None
    row_med = float(np.median(rows))
    res_med = float(np.median(ress))
    offset = forward_offset_from_row(row_med)
    clean = sum(1 for r in ress if r < FORWARD_OFFSET_MAX_RES)
    confident = (res_med < FORWARD_OFFSET_MAX_RES
                 and abs(offset) <= FORWARD_OFFSET_MAX_MM
                 and clean >= (n + 1) // 2)
    return ForwardOffset(offset_mm=offset, row_at_center=row_med,
                         residual_px=res_med, confident=confident)


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


# 正味の物理回転量[deg]の累積(RANG/SANGでodo_angがリセットされても物理回転は
# 追える)。電源ケーブルのよじれ対策(2026-08-03、ユーザー指摘): 同方向に偏って
# 旋回するとケーブルがよじれるので、これを見て左右バランスよく回す/ほどく。
_net_phys_deg = 0.0


def net_rotation_deg() -> float:
    """セッション中の正味物理回転量[deg]。+ = CCW偏り。"""
    return _net_phys_deg


def reset_net_rotation() -> None:
    global _net_phys_deg
    _net_phys_deg = 0.0


def _hold_turn(link, delta_rad: float, *, settle_s: float = 0.8) -> None:
    """相対 delta_rad だけ TURN。stop() は呼ばず保持継続(drift防止)。"""
    global _net_phys_deg
    if abs(delta_rad) < 1e-4:
        return
    link.send(f"TURN,{delta_rad:.5f}")
    time.sleep(settle_s + abs(delta_rad) / 2.5)
    _net_phys_deg += math.degrees(delta_rad)


def _read_ang_deg(link):
    for _ in range(8):
        s = link.read_sen()
        if s:
            return math.degrees(s["odo_ang"])
        time.sleep(0.02)
    return None


def turn_to(link, target_deg: float, *, tol: float = 1.0, tries: int = 4) -> Optional[float]:
    """超信地旋回で odo_ang を target_deg[deg] へ合わせる(hold継続)。

    ケーブルよじれ対策: 最短方向(err)と逆回り(alt=err∓360)がほぼ同じ手数のとき
    (=|err|が180付近で曖昧なとき)は、正味回転(_net_phys_deg)を0へ近づける向きを
    選ぶ。明確に短い旋回では最短を使う(無駄な大回転を避ける)。
    """
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
        # 逆回り候補(|alt| = 360-|err|)。ほぼ同手数(|err|が150°超)かつよじれを
        # 減らすなら逆回りを選ぶ。
        alt = err - 360.0 if err > 0 else err + 360.0
        delta = err
        if abs(err) > 150.0 and abs(_net_phys_deg + alt) < abs(_net_phys_deg + err):
            delta = alt
        _hold_turn(link, math.radians(delta), settle_s=0.6)
        time.sleep(0.15)
    return _read_ang_deg(link)


def unwind_cable(link, *, max_deg: float = 200.0) -> float:
    """電源ケーブルのよじれをほどく: 正味物理回転(_net_phys_deg)を打ち消す向きへ
    超信地旋回する。大回転は壁と干渉し危険なので max_deg でクランプ(必要なら
    複数回呼ぶ)。呼び出し後は heading が変わるので B(reanchor)で貼り直すこと。
    戻り値: 残りの正味回転[deg]。"""
    if abs(_net_phys_deg) < 5.0:
        return _net_phys_deg
    d = max(-max_deg, min(max_deg, -_net_phys_deg))
    _hold_turn(link, math.radians(d))
    link.stop()
    return _net_phys_deg


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
