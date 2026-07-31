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
    set_reload_servo(angle_deg) / set_fan_percent(percent) / read_sensors(timeout_s)

L1(ボール回収)はさらに Futaba アームサーボ(mob とは別のPi UART接続、
software/arm/futaba_servo.py)が要る。RemoteBase とは別に duck-typed な
`arm`(set_angle(angle_deg, move_time_ms=0))をコンストラクタで受け取る
(未接続なら arm=None で、L1は警告してスキップする)。

on_event() はボタンの押下/解放で内部状態を更新するだけ(ネットワーク受信
スレッドから呼ぶ)。実際に base のメソッドを呼ぶのは step()(モーション
実行スレッドから一定間隔で呼ぶ)で、両者は Lock で同期する。これにより
「1区間前進の完了待ち(stop_at のブロッキング)」中でもボタンの押下/解放
イベント自体は取りこぼさない。

十字キー左/右/下(旋回)・L1(ボール回収)・R1(リロード解放)は押すごとに
FIFO キューへ積む(押下イベントそのものを溜め込む)。stop_at()/turn() は
mob 側の DONE 応答(STOP/TURNは完了後0.5秒の角度維持ホールドを経てから
返る)を待つブロッキング呼び出しのため、前の動作の完了待ち中に複数回
押しても、単一変数で最後の1回だけを残す実装だと押下イベントが失われる
(取りこぼし)。キューにすることで、モーション実行スレッドが空くたびに
古い順から確実に1回ずつ消化される(2026-07-25、体感レイテンシの原因調査を
受けて単一変数からキューに変更)。

操作仕様(software/manual_controller/README.md も参照):
    十字キー上   : 押している間、1区間前進を繰り返す。離したら現在の
                   区間の前進が完了(=停止)し次第、次の区間には進まない。
    十字キー右   : 押した瞬間に右90度旋回(1回のみ、離しても何もしない)。
    十字キー左   : 押した瞬間に左90度旋回(同上)。
    十字キー下   : 押した瞬間に180度旋回(同上)。
    △/○/×/▢  : 押している間、低速前進/右旋回/後退/左旋回(LATCH動作)。
                   離したら停止。
    L1           : 押した瞬間にボール回収シーケンス(ball_pickup.py)を1回
                   実行(数秒かかるブロッキング動作)。
    R1           : 押した瞬間にリロードサーボを180度へ(1回のみ)。
"""

from __future__ import annotations

import math
import sys
import threading
from collections import deque
from pathlib import Path
from typing import Callable, Deque, Optional, Protocol, Tuple

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "micromouse"))
from errors import AbortRequested, MobileBaseError  # noqa: E402

import remote_protocol as proto  # noqa: E402
from ball_pickup import BallPickupArm, BallPickupBase, RELOAD_RELEASE_DEG, run_ball_pickup  # noqa: E402

HALF_PI = math.pi / 2.0

# ボタン → 90/180度旋回の角度[rad](正=左/CCW、mob の TURN と同じ符号)
TURN_ANGLES = {
    proto.DPAD_LEFT: +HALF_PI,
    proto.DPAD_RIGHT: -HALF_PI,
    proto.DPAD_DOWN: math.pi,
}

# ボタン → LATCH 動作(押している間だけ低速で動く)の対応
JOG_BUTTONS = frozenset({proto.TRIANGLE, proto.CIRCLE, proto.CROSS, proto.SQUARE})

# アクションキューの種別タグ
ACTION_TURN = "turn"
ACTION_BALL_PICKUP = "ball_pickup"
ACTION_RELOAD_RELEASE = "reload_release"


class RemoteBase(BallPickupBase, Protocol):
    """RemoteController が要求する base の最小インターフェース。"""

    def stop_at(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None: ...
    def turn(self, angle_rad: float) -> None: ...
    def reset_distance(self) -> None: ...
    def latch_forward(self) -> None: ...
    def latch_backward(self) -> None: ...
    def latch_turn_left(self) -> None: ...
    def latch_turn_right(self) -> None: ...
    def latch_stop(self) -> None: ...
    def emergency_stop(self) -> None: ...
    # set_reload_servo/set_fan_percent/read_sensors は BallPickupBase から継承


class RemoteController:
    def __init__(
        self,
        base: RemoteBase,
        *,
        arm: Optional[BallPickupArm] = None,
        cell_speed_mmps: float = 300.0,
        cell_accel_mmps2: float = 1000.0,
        cell_size_mm: float = 180.0,
        on_command_done: Optional[Callable[[], None]] = None,
    ):
        self.base = base
        self.arm = arm
        self.cell_speed_mmps = cell_speed_mmps
        self.cell_accel_mmps2 = cell_accel_mmps2
        self.cell_size_mm = cell_size_mm
        # 1区間前進・90/180度旋回が成功完了するたびに呼ぶコールバック
        # (コントローラの振動フィードバック用、remote_server.py参照)。
        # JOG系(押しっぱなし)・L1・R1では呼ばない。
        self.on_command_done = on_command_done

        self.ball_held = False  # L1シーケンス成功で True(OLED表示等に利用)

        self._lock = threading.Lock()
        self._dpad_up_held = False
        self._jog_active: Optional[str] = None       # on_event() が更新
        # 旋回・L1・R1 の押下イベントをFIFOで保持: (種別タグ, ペイロード)
        self._action_queue: Deque[Tuple[str, object]] = deque()

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
                    self._action_queue.append((ACTION_TURN, TURN_ANGLES[name]))
            return

        if name == proto.L1:
            if pressed:
                with self._lock:
                    self._action_queue.append((ACTION_BALL_PICKUP, None))
            return

        if name == proto.R1:
            if pressed:
                with self._lock:
                    self._action_queue.append((ACTION_RELOAD_RELEASE, None))
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

        action = self._pop_queued_action()
        if action is not None:
            self._dispatch_action(action)
            return

        with self._lock:
            dpad_up_held = self._dpad_up_held
        if dpad_up_held:
            # mob の STOP は距離を累積目標(cumulative_goal_dist_mm)に
            # 加算する方式で、JOG系コマンド(JOGFWD/JOGBACK)はこの累積値を
            # 更新しない。そのため直前にJOGで動いていたり、過去のFWD/STOPの
            # 実移動量が指令値とわずかにずれていたりすると、そのズレを
            # 引きずったまま「180mm前進」のつもりが違う距離になる。
            # 毎回 RDST(距離リセット、cumulative_goal_dist_mmも同時に
            # リセットされる)してから STOP することで、常に現在位置から
            # 正確に cell_size_mm だけ進むようにする。
            if self._safe_call(self.base.reset_distance):
                ok = self._safe_call(
                    self.base.stop_at, self.cell_speed_mmps, self.cell_accel_mmps2, self.cell_size_mm
                )
                if ok:
                    self._notify_command_done()
            return

    def handle_disconnect(self) -> None:
        """接続断・異常検出時に呼ぶ: 緊急停止し内部状態を初期化する。

        ball_held(物理的にボールを保持しているかの状態)は接続断で変わる
        ものではないため初期化しない。
        """
        self._safe_call(self.base.emergency_stop)
        with self._lock:
            self._dpad_up_held = False
            self._jog_active = None
            self._action_queue.clear()
        self._jog_sent = None

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _pop_queued_action(self) -> Optional[Tuple[str, object]]:
        with self._lock:
            if self._action_queue:
                return self._action_queue.popleft()
            return None

    def _dispatch_action(self, action: Tuple[str, object]) -> None:
        kind, payload = action
        if kind == ACTION_TURN:
            if self._safe_call(self.base.turn, payload):
                self._notify_command_done()
        elif kind == ACTION_BALL_PICKUP:
            self._run_ball_pickup()
        elif kind == ACTION_RELOAD_RELEASE:
            self._safe_call(self.base.set_reload_servo, RELOAD_RELEASE_DEG)

    def _run_ball_pickup(self) -> None:
        if self.arm is None:
            print("# RemoteController: アームサーボ未接続のためL1シーケンスをスキップ")
            return
        try:
            self.ball_held = run_ball_pickup(self.base, self.arm)
        except Exception as e:
            # アームサーボ(Futaba)はmobとは別のシリアル接続で、
            # AbortRequested/MobileBaseError以外の例外(pyserial由来等)も
            # 起こりうるため、ここは広く捕捉してワーカースレッドを守る。
            print(f"# RemoteController: L1シーケンス中にエラー: {e}")

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

    def _safe_call(self, fn, *args) -> bool:
        """base 呼び出しの例外を吸収する(リンク切断による中断など)。

        通信スレッドの watchdog が handle_disconnect() を呼んで安全停止
        させる設計なので、ここでは実行を継続できるようにログのみ行う。
        戻り値: 例外無く完了したら True(振動通知など、成功時のみ行う
        処理の判断に使う)。
        """
        try:
            fn(*args)
            return True
        except (AbortRequested, MobileBaseError) as e:
            print(f"# RemoteController: 動作中断({fn.__name__}): {e}")
            return False

    def _notify_command_done(self) -> None:
        if self.on_command_done is None:
            return
        try:
            self.on_command_done()
        except Exception as e:
            print(f"# RemoteController: on_command_done コールバックでエラー: {e}")
