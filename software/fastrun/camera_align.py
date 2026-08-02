"""camera_align.py — 走行前カメラ初期姿勢補正(Phase 2、2026-08-03〜)。

搭載カメラ(Picamera2、前方固定 Futabaサーボ角0度)で前壁の赤帯上端エッジを
検出し、機体のヨー角ズレ・前壁までの距離オフセットを推定する。ヨー角は
現行ファームの TURN(その場旋回)+ SANG(積分角度の絶対上書き)で物理的に
真っ直ぐへ補正する。距離オフセットは走行計画の最初の直進へ織り込む方針
(現行ファームに精密な微小前進プリミティブが無いため。JOGFWD等は
2026-08-02に削除済み)。

推定ロジック・較正定数は削除済み software/micromouse/camera_correction.py
(git d75afa7、2026-07-24に実機較正)から移植。ただし補正の実行手段
(jog_turn/jog_forward/correct_angle)は現行ファームに無いため作り直した。

⚠️ 較正定数は特定のカメラ解像度(2304x1296)・サーボ角0・マウント位置で
フィットしたもの。撮影は必ず同解像度・中央50%クロップで行うこと。マウントを
触った後は estimate_pose の符号・スケールを実機で再検証してから補正に使う。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from vision_wall import detect_red_band_top_edge

# --- ヨー角較正(2026-08-03、ジャイロを真値にしたスロープ・スイープで再較正) ---
# 手法: 前壁に正対した状態を基準(RANG=0)に、TURNで既知の相対角(-8〜0deg)へ
# 回しながら赤帯上端エッジの傾き slope_deg=degrees(atan(edge.slope)) を記録し、
# slope_deg = SLOPE_DEG_PER_YAW_DEG * yaw_deg + STRAIGHT_SLOPE_DEG を最小二乗フィット
# (res<2pxの清浄点のみ)。yaw_deg は「壁への正対からのズレ角[deg]、正=左/CCW」。
#   => 正対(yaw=0)へ戻すには TURN で -yaw_deg 回す。
# 旧micromouse較正(gain相当0.532/2026-07-24の9点)はこの機体・距離では約2倍
# ずれており、旧定数で「yaw=0へ補正」すると逆に約4°ずらしていた(実機確認)。
# ⚠️ ゲインは壁までの距離に依存する(近いほど同じ角度で傾きが大きく出る)。
# この値は「前壁まで約1セル」で取得。別距離で使うなら要再較正(距離依存の
# モデル化は今後の課題)。左(+)へ向けると中央クロップに隣の壁が混入して
# フィットが壊れる(res急増)ため、負側+0付近の清浄点で較正した。
SLOPE_DEG_PER_YAW_DEG = 0.248   # d(slope_deg)/d(yaw_deg) @ 約1セル
STRAIGHT_SLOPE_DEG = 0.203      # 正対(yaw=0)時の赤帯slope[deg]

# --- 距離較正(暫定・要再較正) ---
# 旧micromouse定数のまま。dist_offset_mm は「基準距離からの前進オフセット」で、
# ヨー角較正と同様この機体・距離では未検証。当面は計測・ログのみに使い、
# 走行計画へ織り込む距離補正には使わない(2026-08-03)。
CAMERA_DIST_GAIN_PX_PER_MM = 11.599
CAMERA_DIST_INTERCEPT_PX = 691.55

# 信頼度ゲート(residual が主。左向きで別の壁が混入した不良フィットを弾く)
CAMERA_CORRECTION_MAX_RESIDUAL_PX = 2.0
CAMERA_CORRECTION_MAX_YAW_DEG = 15.0
CAMERA_CORRECTION_MAX_DIST_MM = 40.0

# 較正時の撮影解像度(この解像度でないと画素単位の較正式が合わない)
CALIB_WIDTH = 2304
CALIB_HEIGHT = 1296


@dataclass(frozen=True)
class PoseEstimate:
    yaw_deg: float          # 正=機体が左(CCW)を向いている
    dist_offset_mm: float   # 正=基準より前進方向へ余分にいる
    residual_px: float
    inlier_count: int


def estimate_pose(img: np.ndarray) -> Optional[PoseEstimate]:
    """RGB画像(H,W,3 uint8)から推定ヨー角・距離オフセットを求める。

    中央50%クロップ(画面端の別の壁を巻き込まないため)→ 赤帯上端エッジ検出。
    検出できなければ None。
    """
    h, w, _ = img.shape
    cropped = np.ascontiguousarray(img[:, w // 4 : 3 * w // 4, :])

    edge = detect_red_band_top_edge(cropped)
    if edge is None:
        return None

    slope_deg = math.degrees(math.atan(edge.slope))
    yaw_deg = (slope_deg - STRAIGHT_SLOPE_DEG) / SLOPE_DEG_PER_YAW_DEG

    cropped_width = cropped.shape[1]
    row_at_center = edge.slope * (cropped_width / 2) + edge.intercept
    dist_offset_mm = (
        row_at_center - CAMERA_DIST_INTERCEPT_PX
    ) / CAMERA_DIST_GAIN_PX_PER_MM

    return PoseEstimate(
        yaw_deg=yaw_deg,
        dist_offset_mm=dist_offset_mm,
        residual_px=edge.residual_std,
        inlier_count=edge.inlier_count,
    )


def is_confident(estimate: PoseEstimate) -> bool:
    """較正範囲から大きく外れた(=信頼できない)推定値を弾く。"""
    return (
        estimate.residual_px <= CAMERA_CORRECTION_MAX_RESIDUAL_PX
        and abs(estimate.yaw_deg) <= CAMERA_CORRECTION_MAX_YAW_DEG
        and abs(estimate.dist_offset_mm) <= CAMERA_CORRECTION_MAX_DIST_MM
    )


class OnboardCamera:
    """搭載カメラ(Picamera2)+ 前方固定サーボ。撮影のみを担う。

    初期化コストが大きいため1度だけ生成して使い回す(camera_test.pyと同方針)。
    サーボは論理角0度(前方固定)にしてカメラの向きを走行時と揃える。
    """

    def __init__(self, width: int = CALIB_WIDTH, height: int = CALIB_HEIGHT,
                 move_servo: bool = True):
        import sys
        from pathlib import Path

        sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "arm"))
        self._servo = None
        if move_servo:
            from futaba_servo import FutabaServo

            self._servo = FutabaServo()
            self._servo.set_torque(True)
            self._servo.set_angle(0.0, move_time_ms=500)
            time.sleep(0.8)

        from picamera2 import Picamera2

        self._cam = Picamera2()
        cfg = self._cam.create_still_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._cam.configure(cfg)
        self._cam.start()
        time.sleep(1.0)  # 露出安定待ち

    def capture(self) -> np.ndarray:
        """1枚撮影して RGB(H,W,3) を返す。"""
        array = self._cam.capture_array("main")
        return array[:, :, ::-1]  # RGB888 の capture_array は numpy 上 BGR 並び

    def close(self) -> None:
        try:
            self._cam.stop()
        except Exception:
            pass
        if self._servo is not None:
            self._servo.close()

    def __enter__(self) -> "OnboardCamera":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _turn_by(link, turn_rad: float, *, settle_s: float = 0.6) -> None:
    """TURN で相対 turn_rad だけ回し、旋回時間ぶん待ってから停止する。"""
    link.send(f"TURN,{turn_rad:.5f}")
    time.sleep(settle_s + abs(turn_rad) / 3.0)  # おおまかな旋回時間ぶん余裕
    link.stop()  # MOT,0,0 で TURN の継続角度保持を止める
    time.sleep(0.15)


def align_heading(link, pose: PoseEstimate, *, settle_s: float = 0.6) -> None:
    """推定ヨー角ぶんだけ TURN で真っ直ぐへ回し、SANG,0 で基準化する(単発)。

    pose.yaw_deg は「正=機体が壁への正対から左(CCW)を向いている」ので、
    正対へ戻すには -yaw 回す。TURN は完了通知を返さないので時間で待つ。
    """
    _turn_by(link, -math.radians(pose.yaw_deg), settle_s=settle_s)
    link.send("SANG,0")
    link.wait_for("DONE", timeout_s=1.0)


def align_to_wall(
    link,
    cam: "OnboardCamera",
    *,
    iterations: int = 3,
    deadband_deg: float = 1.0,
    max_step_deg: float = 20.0,
    set_reference: bool = True,
) -> Optional[PoseEstimate]:
    """前壁に正対するまでカメラ推定→TURN補正を反復する閉ループ(2026-08-03)。

    毎回撮影して estimate_pose し、is_confident でなければ補正を諦める
    (別の壁を掴んだ不良フィット等)。|yaw|<deadband_deg なら整定とみなし終了。
    それ以外は -yaw(max_step_degでクランプ)だけ TURN で回す。最後に
    set_reference なら SANG,0 で「正対=角度0」を走行の基準に定める。

    戻り値は最後の推定値(ログ用)。1回で約9°→1°、数回でサブ度まで収束
    (実機確認、2026-08-03)。壁までの距離が較正時(約1セル)と大きく違うと
    ゲインがずれる点、機体が左を向くと中央クロップに隣の壁が混入して
    detection が壊れ is_confident に弾かれ補正できない点が既知の限界。
    """
    last: Optional[PoseEstimate] = None
    for _ in range(iterations):
        est = estimate_pose(cam.capture())
        last = est
        if est is None or not is_confident(est):
            break  # 信頼できない推定では補正しない(安全側)
        if abs(est.yaw_deg) <= deadband_deg:
            break  # 整定
        step_deg = max(-max_step_deg, min(max_step_deg, est.yaw_deg))
        _turn_by(link, -math.radians(step_deg))
    if set_reference:
        link.send("SANG,0")
        link.wait_for("DONE", timeout_s=1.0)
    return last
