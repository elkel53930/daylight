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

# --- 較正定数(2026-07-24、micromouse camera-sweep 実機較正) ---
CAMERA_YAW_SLOPE_GAIN = 0.5322
CAMERA_YAW_SLOPE_INTERCEPT_DEG = 3.1225
CAMERA_YAW_BIAS_DEG = 2.0  # 補正後の系統残差(正=左/CCW)を打ち消す最終トリム

CAMERA_DIST_GAIN_PX_PER_MM = 11.599
CAMERA_DIST_INTERCEPT_PX = 691.55

# 信頼度ゲート(較正範囲±10度に対しマージンを持たせた値)
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
    yaw_deg = (
        slope_deg - CAMERA_YAW_SLOPE_INTERCEPT_DEG
    ) / CAMERA_YAW_SLOPE_GAIN + CAMERA_YAW_BIAS_DEG

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


def align_heading(link, pose: PoseEstimate, *, settle_s: float = 0.6) -> None:
    """推定ヨー角ぶんだけ TURN で物理的に真っ直ぐへ回し、SANG,0 で基準化する。

    link は MobLink(.send/.wait_for)。TURN の符号は「正=左(CCW)」。
    pose.yaw_deg は「正=機体が左を向いている」なので、真っ直ぐへ戻すには
    -yaw だけ回す(=右へ|yaw|)。TURN は完了通知を返さないので settle_s 待つ。
    """
    turn_rad = -math.radians(pose.yaw_deg)
    link.send(f"TURN,{turn_rad:.5f}")
    time.sleep(settle_s + abs(turn_rad) / 3.0)  # おおまかな旋回時間ぶん余裕を足す
    link.stop()  # MOT,0,0 で TURN の継続角度保持を止める
    time.sleep(0.1)
    link.send("SANG,0")
    link.wait_for("DONE", timeout_s=1.0)
