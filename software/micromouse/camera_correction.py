"""カメラ(壁上面の赤帯検出)によるヨー角・前進距離の補正。

software/micromouse/vision.py の赤帯上端エッジ検出を使い、機体の現在の
ヨー角・前進軸(現在向いている方向)の距離ズレを推定して mob 側の状態を
補正する。較正定数・推定ロジックは hw_test.py の camera-correct/
camera-straighten コマンドで実機検証したもの(2026-07-24)をそのまま使う。
探索走行本体(state_machine.py)と手動デモ(hw_test.py)の両方から使う
共通モジュール。

カメラは前方固定(Futabaサーボ論理角度0度、software/arm/futaba_servo.py)
のため、補正できるのは「現在向いている軸」のみ。機体は探索中に東西南北
すべてを向くため、セッション全体で見ればX軸・Y軸どちらも別々のタイミング
で補正される(側壁を同時検出して垂直方向も同時に補正する仕組みは較正
データが無く未実装、TODO.md参照)。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from vision import detect_red_band_top_edge

# 較正定数(2026-07-24、camera-sweepで撮った9点のみの経験的フィット)。
# slope_deg = GAIN * yaw_deg + INTERCEPT_DEG の逆算に使う。
# サンプル数が少なく、横位置ズレとの分離もできていない暫定値
# (TODO.md「micromouse: カメラ角度補正(vision.py)の較正精度」参照)。
CAMERA_YAW_SLOPE_GAIN = 0.5322
CAMERA_YAW_SLOPE_INTERCEPT_DEG = 3.1225

# camera-straightenで複数回試したところ、補正後に目視で毎回左に約1.5度
# ズレて止まっていた(2026-07-24)。上の較正式(GAIN/INTERCEPT)はそのまま
# にして、この系統的な残差だけを打ち消す最終トリム値として別に持つ
# (較正式を再フィットしたら本来はここも0に近づくはず)。
# 符号: 正=左(CCW)方向へのズレ。推定ヨー角に加算してから補正量を計算する。
CAMERA_YAW_BIAS_DEG = 2.0

# 較正定数(2026-07-24、camera-sweep-distanceで撮った4点のみの経験的フィット)。
# row_at_center = GAIN_PX_PER_MM * dist_mm + INTERCEPT_PX の逆算に使う。
# dist_mm は基準位置(reset_distance()した時点)からの前進距離。
# サンプル数が少なく、ヨー角との分離もできていない暫定値
# (TODO.md「micromouse: カメラ角度補正(vision.py)の較正精度」参照)。
CAMERA_DIST_GAIN_PX_PER_MM = 11.599
CAMERA_DIST_INTERCEPT_PX = 691.55

# 信頼度ゲート(2026-07-24 実機で確認: 較正範囲±10度を大きく超えた状態
# から補正すると、残差自体はクリーンに見えても間違った角度に収束する
# ことがあった。較正範囲に対して十分なマージンを持たせた値。これを
# 超える推定値は「別の壁を追跡してしまった」等で信頼できないとみなし、
# その軸の補正をスキップする)。
CAMERA_CORRECTION_MAX_RESIDUAL_PX = 2.0
CAMERA_CORRECTION_MAX_YAW_DEG = 15.0
CAMERA_CORRECTION_MAX_DIST_MM = 40.0


@dataclass(frozen=True)
class PoseEstimate:
    """1回の撮影から推定したヨー角・前進距離オフセット。"""

    yaw_deg: float
    dist_offset_mm: float
    residual_px: float
    inlier_count: int


def estimate_pose(img: np.ndarray) -> Optional[PoseEstimate]:
    """RGB画像(H,W,3 uint8)から推定ヨー角・距離オフセットを求める。

    中央50%クロップ(画面端の別の壁を巻き込まないため、実機画像で確認済み)
    → vision.detect_red_band_top_edge。検出できなければ None。
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
    dist_offset_mm = (row_at_center - CAMERA_DIST_INTERCEPT_PX) / CAMERA_DIST_GAIN_PX_PER_MM

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


class CameraCorrector:
    """カメラ・Futabaサーボを保持し、mobの角度・距離を補正する。

    Picamera2/FutabaServoの初期化はコストが大きいため、ミッション開始時に
    1度だけ生成し(camera_test.pyのストリーミング方式と同じ)、以後の
    補正のたびに capture() を呼ぶだけにする。
    """

    def __init__(self, width: int = 2304, height: int = 1296):
        import sys

        sys.path.insert(0, str(Path(__file__).parent.parent / "arm"))
        from futaba_servo import FutabaServo
        from picamera2 import Picamera2

        self._servo = FutabaServo()
        self._servo.set_torque(True)
        self._servo.set_angle(0.0, move_time_ms=500)
        time.sleep(0.8)  # サーボ静定待ち

        self._cam = Picamera2()
        still_config = self._cam.create_still_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._cam.configure(still_config)
        self._cam.start()
        time.sleep(1.0)  # 露出安定待ち

    def capture(self) -> np.ndarray:
        array = self._cam.capture_array("main")
        return array[:, :, ::-1]  # BGR -> RGB

    def try_correct(self, base) -> Optional[PoseEstimate]:
        """角度→距離の順で1回分の補正を試みる。

        角度がズレたままだと距離推定も乱れるため、角度を補正してから
        改めて撮影して距離を推定する(hw_test.py camera-straightenと
        同じ手順)。それぞれ is_confident() を満たさなければその軸は
        スキップする。戻り値は最後に撮影した推定値(ログ用)。
        両方とも検出・信頼できなければ None。
        """
        yaw_estimate = estimate_pose(self.capture())
        if yaw_estimate is None:
            return None

        if is_confident(yaw_estimate):
            base.jog_turn(math.radians(-yaw_estimate.yaw_deg))
            base.correct_angle(0.0)  # ここを新しい基準(まっすぐ)とする
            time.sleep(0.3)  # 露出再安定待ち

        dist_estimate = estimate_pose(self.capture())
        if dist_estimate is not None and is_confident(dist_estimate):
            if dist_estimate.dist_offset_mm > 0:
                base.jog_forward(dist_estimate.dist_offset_mm)
            elif dist_estimate.dist_offset_mm < 0:
                base.jog_backward(abs(dist_estimate.dist_offset_mm))
            base.reset_distance()  # ここを新しい基準距離とする

        return dist_estimate if dist_estimate is not None else yaw_estimate

    def close(self) -> None:
        try:
            self._cam.stop()
        except Exception:
            pass
        # FutabaServo自身は__del__でトルクオフされるが、明示的にも閉じておく
        self._servo.close()
