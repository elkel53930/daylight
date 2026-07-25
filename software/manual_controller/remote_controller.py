"""ボタンイベント→機体動作の変換ロジック(ネットワーク・シリアル非依存)。

remote_server.py から呼ばれる中核クラス RemoteController を提供する。
テスト容易性のため、実際の MobileBase ではなく duck-typed な `base`
オブジェクト(以下のメソッドを持てば何でもよい)を受け取る:

    stop_at(speed_mmps, accel_mmps2, distance_mm)   # 加減速して1区間走行→停止
    turn(angle_rad)                                  # その場旋回(正=左/CCW)
    latch_forward() / latch_backward()
    latch_turn_left() / latch_turn_right()
    latch_stop()
    emergency_stop()

on_event() はボタンの押下/解放で内部状態を更新するだけ(ネットワーク受信
スレッドから呼ぶ)。実際に base のメソッドを呼ぶのは step()(モーション
実行スレッドから一定間隔で呼ぶ)で、両者は Lock で同期する。これにより
「1区間前進の完了待ち(stop_at のブロッキング)」中でもボタンの押下/解放
イベント自体は取りこぼさない。

操作仕様(software/manual_controller/README.md も参照):
    十字キー上   : 押している間、1区間前進を繰り返す。離したら現在の
                   区間の前進が完了(=停止)し次第、次の区間には進まない。
    十字キー右   : 押した瞬間に右90度旋回(1回のみ、離しても何もしない)。
    十字キー左   : 押した瞬間に左90度旋回(同上)。
    十字キー下   : 押した瞬間に180度旋回(同上)。
    △/○/×/▢  : 押している間、低速前進/右旋回/後退/左旋回(LATCH動作)。
                   離したら停止。
    L1/R1        : 未実装(将来: アーム・リロードサーボ・ボールセンサ操作)。
"""

from __future__ import annotations

import math
import sys
import threading
from pathlib import Path
from typing import Optional, Protocol

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "micromouse"))
from errors import AbortRequested, MobileBaseError  # noqa: E402

import protocol as proto  # noqa: E402

HALF_PI = math.pi / 2.0

# ボタン → 90/180度旋回の角度[rad](正=左/CCW、mob の TURN と同じ符号)
TURN_ANGLES = {
    proto.DPAD_LEFT: +HALF_PI,
    proto.DPAD_RIGHT: -HALF_PI,
    proto.DPAD_DOWN: math.pi,
}

# ボタン → LATCH 動作(押している間だけ低速で動く)の対応
JOG_BUTTONS = frozenset({proto.TRIANGLE, proto.CIRCLE, proto.CROSS, proto.SQUARE})


class RemoteBase(Protocol):
    """RemoteController が要求する base の最小インターフェース。"""

    def stop_at(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None: ...
    def turn(self, angle_rad: float) -> None: ...
    def latch_forward(self) -> None: ...
    def latch_backward(self) -> None: ...
    def latch_turn_left(self) -> None: ...
    def latch_turn_right(self) -> None: ...
    def latch_stop(self) -> None: ...
    def emergency_stop(self) -> None: ...


class RemoteController:
    def __init__(
        self,
        base: RemoteBase,
        *,
        cell_speed_mmps: float = 300.0,
        cell_accel_mmps2: float = 1000.0,
        cell_size_mm: float = 180.0,
    ):
        self.base = base
        self.cell_speed_mmps = cell_speed_mmps
        self.cell_accel_mmps2 = cell_accel_mmps2
        self.cell_size_mm = cell_size_mm

        self._lock = threading.Lock()
        self._dpad_up_held = False
        self._jog_active: Optional[str] = None       # on_event() が更新
        self._pending_turn_rad: Optional[float] = None

        self._jog_sent: Optional[str] = None          # step() 専用(単一スレッドのみ触る)

    # ------------------------------------------------------------------
    # ネットワーク受信スレッドから呼ぶ: 状態更新のみ(base は呼ばない)
    # ------------------------------------------------------------------

    def on_event(self, name: str, action: str) -> None:
        pressed = action == proto.ACTION_DOWN

        if name == proto.DPAD_UP:
            with self._lock:
                self._dpad_up_held = pressed
            return

        if name in TURN_ANGLES:
            if pressed:
                with self._lock:
                    self._pending_turn_rad = TURN_ANGLES[name]
            return

        if name in JOG_BUTTONS:
            with self._lock:
                if pressed:
                    self._jog_active = name
                elif self._jog_active == name:
                    # 押されているボタンが切り替わっている場合、無関係な
                    # ボタンの解放で誤って止めない(現在アクティブなボタン
                    # 自身の解放のときだけ止める)。
                    self._jog_active = None
            return

        if name in (proto.L1, proto.R1):
            return  # TODO: アーム/リロードサーボ/ボールセンサ操作

        # 未知ボタンは無視

    # ------------------------------------------------------------------
    # モーション実行スレッドから一定間隔で呼ぶ: 1 tick で最大1アクション
    # ------------------------------------------------------------------

    def step(self) -> None:
        with self._lock:
            jog = self._jog_active
        if jog != self._jog_sent:
            self._dispatch_jog(jog)
            self._jog_sent = jog
            return

        turn = self._pop_pending_turn()
        if turn is not None:
            self._safe_call(self.base.turn, turn)
            return

        with self._lock:
            dpad_up_held = self._dpad_up_held
        if dpad_up_held:
            self._safe_call(
                self.base.stop_at, self.cell_speed_mmps, self.cell_accel_mmps2, self.cell_size_mm
            )
            return

    def handle_disconnect(self) -> None:
        """接続断・異常検出時に呼ぶ: 緊急停止し内部状態を初期化する。"""
        self._safe_call(self.base.emergency_stop)
        with self._lock:
            self._dpad_up_held = False
            self._jog_active = None
            self._pending_turn_rad = None
        self._jog_sent = None

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _pop_pending_turn(self) -> Optional[float]:
        with self._lock:
            t = self._pending_turn_rad
            self._pending_turn_rad = None
            return t

    def _dispatch_jog(self, name: Optional[str]) -> None:
        if name is None:
            self._safe_call(self.base.latch_stop)
        elif name == proto.TRIANGLE:
            self._safe_call(self.base.latch_forward)
        elif name == proto.CIRCLE:
            self._safe_call(self.base.latch_turn_right)
        elif name == proto.CROSS:
            self._safe_call(self.base.latch_backward)
        elif name == proto.SQUARE:
            self._safe_call(self.base.latch_turn_left)

    def _safe_call(self, fn, *args) -> None:
        """base 呼び出しの例外を吸収する(リンク切断による中断など)。

        通信スレッドの watchdog が handle_disconnect() を呼んで安全停止
        させる設計なので、ここでは実行を継続できるようにログのみ行う。
        """
        try:
            fn(*args)
        except (AbortRequested, MobileBaseError) as e:
            print(f"# RemoteController: 動作中断({fn.__name__}): {e}")
