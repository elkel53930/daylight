"""センサ値の解釈(SEN パースと壁判定)。

処理段階:
    raw SEN line → SensorFrame(パース) → WallDetector(しきい値) → WallObservation

Daylight の壁センサは 前1 + 左横 + 右横 の3個。mob の SEN 応答は
Twilight 互換のため lf/rf の両方に前センサ値が入る(sensors.cpp 参照)。
本モジュールでは front = min(lf, rf) を前センサ値として扱う
(Daylight では lf == rf なのでそのまま、仮に将来 2 個に戻っても
「両方がしきい値以上で前壁」という Twilight の判定と等価になる)。

しきい値はセンサ個体・迷路の材質に依存するため config で調整する。
校正には hw_test.py の walls サブコマンドを使う。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from explorer import WallObservation


@dataclass(frozen=True)
class SensorFrame:
    """mob の SEN 応答 1 行分。

    SEN,<gyro rad/s>,<batt V>,<lf>,<ls>,<rs>,<rf>,<enc_r>,<enc_l>,<dist mm>,<ang rad>,<ball_raw>,<ball_det>

    ball_raw/ball_det は迷路走行では未使用だが、mob.ino の SEN 応答に
    含まれるためパース対象として受け取る(桁数チェックのみに使う)。
    """

    gyro_radps: float
    vbatt: float
    lf: int
    ls: int
    rs: int
    rf: int
    enc_r: int
    enc_l: int
    odo_dist_mm: float
    odo_ang_rad: float


def parse_sen_line(line: str) -> Optional[SensorFrame]:
    """SEN 行をパースする。形式不正なら None(呼び出し側でリトライ)。"""
    parts = line.strip().split(",")
    if len(parts) != 13 or parts[0] != "SEN":
        return None
    try:
        return SensorFrame(
            gyro_radps=float(parts[1]),
            vbatt=float(parts[2]),
            lf=int(parts[3]),
            ls=int(parts[4]),
            rs=int(parts[5]),
            rf=int(parts[6]),
            enc_r=int(parts[7]),
            enc_l=int(parts[8]),
            odo_dist_mm=float(parts[9]),
            odo_ang_rad=float(parts[10]),
        )
    except ValueError:
        return None


class WallDetector:
    """壁センサ差分値のしきい値判定。

    センサ値は mob 側で LED ON/OFF 差分を取った値なので環境光の影響は
    受けにくいが、壁までの距離とセンサ個体差の影響は残る。しきい値は
    「セル境界(判断点)にロボットがいるとき」の値で校正すること。
    """

    def __init__(
        self,
        *,
        left_threshold: int,
        right_threshold: int,
        front_threshold: int,
        saturation: int = 4095,
    ):
        self.left_threshold = left_threshold
        self.right_threshold = right_threshold
        self.front_threshold = front_threshold
        self.saturation = saturation

    def detect(self, frame: SensorFrame) -> WallObservation:
        return WallObservation(
            left=frame.ls >= self.left_threshold,
            front=min(frame.lf, frame.rf) >= self.front_threshold,
            right=frame.rs >= self.right_threshold,
        )

    def is_sensor_sane(self, frame: SensorFrame) -> bool:
        """センサ異常の簡易チェック(飽和・負値)。"""
        values = (frame.lf, frame.ls, frame.rs, frame.rf)
        return all(0 <= v <= self.saturation for v in values)
