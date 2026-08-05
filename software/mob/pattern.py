"""
pattern.py — mob の PATTERN 走行パス(PCLEAR/PADD/PRUN)をPCから構築・送信する
ためのヘルパー(2026-08-02〜)。

以前は mob.ino 側に固定パスをハードコードしていたが、PC側から任意の
パスを送れるように変更した。角度は無線越しの桁数節約のため度(deg)で
指定し、mob.ino側でradへ変換する。

区間は Straight(直進、台形速度プロファイル)と Slalom(定速円弧旋回)の
2種類。将来的にパターンの種類が増える想定のため、Segment を基底とした
dataclass構成にしてある。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Union


@dataclass(frozen=True)
class Straight:
    distance_mm: float
    v_start_mmps: float
    v_cruise_mmps: float
    v_end_mmps: float

    def to_command(self) -> str:
        return (
            f"PADD,STRAIGHT,{self.distance_mm},{self.v_start_mmps},"
            f"{self.v_cruise_mmps},{self.v_end_mmps}"
        )


@dataclass(frozen=True)
class Slalom:
    v_mmps: float
    dir: str  # "L"(左/CCW) または "R"(右/CW)
    radius_mm: float
    angle_deg: float

    def __post_init__(self) -> None:
        if self.dir not in ("L", "R"):
            raise ValueError(f"dir must be 'L' or 'R', got {self.dir!r}")

    def to_command(self) -> str:
        return f"PADD,SLALOM,{self.v_mmps},{self.dir},{self.radius_mm},{self.angle_deg}"


Segment = Union[Straight, Slalom]


def send_pattern(link, segments: Sequence[Segment], *, done_timeout_s: float = 1.0) -> None:
    """PCLEAR → PADD,... (各区間) → PRUN の順に送信する。

    link は `.send(str)` と `.wait_for(prefix, timeout_s)` を持つオブジェクト
    (motion_test.py の MobLink 等)を想定。各コマンドの DONE 応答を待ってから
    次を送る(取りこぼし防止)。
    """
    link.send("PCLEAR")
    link.wait_for("DONE", timeout_s=done_timeout_s)
    for seg in segments:
        link.send(seg.to_command())
        link.wait_for("DONE", timeout_s=done_timeout_s)
    link.send("PRUN")


# 既定テストパス(2026-08-02): 150mm前進 → 90mm前進 → 右90°スラローム×2
# (=右180°Uターン) → 左90°スラローム×2(=左180°Uターン) → 120mm前進(停止)。
# 半径90mm・巡航390mm/s(基準300の1.3倍)。機体の向きが進行方向から90°を超えて
# 回る180°Uターンを含むが、位置復元力(path_controller.cppのベアリングブレンド
# +追従ゲート)により発散せず完走することを実機確認済み(反転区間の追従誤差
# distは最大42mmで頭打ち→回復)。速度は600(2倍)だと精度が大きく低下、
# 390(1.3倍)が実用的なバランスと確認して390で確定。制御パラメータの微調整は継続。
# 注: 180°を単一のPADD,SLALOM(angle_deg=180)で送ると、90°×2に分けた場合と
# ターゲット軌道は同じでも追従が破綻しやすいため、90°×2に分割している。

FWD_SPEED = 300.0

START = Straight(distance_mm=60.0, v_start_mmps=0.0, v_cruise_mmps=FWD_SPEED, v_end_mmps=FWD_SPEED)
FWD = Straight(distance_mm=180.0, v_start_mmps=FWD_SPEED, v_cruise_mmps=FWD_SPEED, v_end_mmps=FWD_SPEED)
RIGHT90_SLALOM = Slalom(v_mmps=FWD_SPEED, dir="R", radius_mm=90.0, angle_deg=90.0)
LEFT90_SLALOM = Slalom(v_mmps=FWD_SPEED, dir="L", radius_mm=90.0, angle_deg=90.0)
STOP = Straight(distance_mm=120.0, v_start_mmps=FWD_SPEED, v_cruise_mmps=FWD_SPEED, v_end_mmps=0.0)

DEFAULT_TEST_PATTERN: tuple[Segment, ...] = (
    START,
    FWD,
    RIGHT90_SLALOM,
    FWD,
    FWD,
    RIGHT90_SLALOM,
    FWD,
    RIGHT90_SLALOM,
    RIGHT90_SLALOM,
    LEFT90_SLALOM,
    RIGHT90_SLALOM,
    LEFT90_SLALOM,
    LEFT90_SLALOM,
    FWD,
    STOP,
)
