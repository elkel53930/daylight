"""PC 側の、pygame 非依存な入出力ロジック。

pygame 自体には依存しない(値の変換ロジックのみ)ので、PC 実機が無くても
ユニットテストできる(remote_client.py は pygame を無条件 import するため、
テスト対象のロジックはこちらに置く)。

- 入力: pygame のジョイスティック入力 → protocol.py のボタン名への変換。
  DualSense を pygame で開いた場合の実測マッピング
  (software/manual_controller/dualsense_test.py で確認済み)を使う。
- 出力: 機体からの受信データの行分割・振動(rumble)通知の解釈。
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import remote_protocol as proto

# pygame の JOYBUTTONDOWN/UP の event.button → protocol のボタン名
# (dualsense_test.py の BUTTON_NAMES と同じ実測マッピング。実機のDualSenseで
# 2026-07-25 に確認: index 2=△, 3=▢。当初 2=▢, 3=△ と誤って想定していた)
BUTTON_INDEX_TO_NAME = {
    0: proto.CROSS,
    1: proto.CIRCLE,
    2: proto.TRIANGLE,
    3: proto.SQUARE,
    4: proto.L1,
    5: proto.R1,
}

HatValue = Tuple[int, int]


def hat_to_events(prev: HatValue, cur: HatValue) -> List[Tuple[str, str]]:
    """十字キー(pygame の hat 値、(x,y))の変化をボタンイベント列に変換する。

    x: -1=左, +1=右, 0=どちらも解放。y: +1=上, -1=下, 0=どちらも解放。
    斜め入力(例: (1,1))は上下それぞれのイベントを個別に発行する。
    """
    events: List[Tuple[str, str]] = []
    prev_x, prev_y = prev
    cur_x, cur_y = cur

    if prev_x != cur_x:
        if prev_x == -1:
            events.append((proto.DPAD_LEFT, proto.ACTION_UP))
        elif prev_x == 1:
            events.append((proto.DPAD_RIGHT, proto.ACTION_UP))
        if cur_x == -1:
            events.append((proto.DPAD_LEFT, proto.ACTION_DOWN))
        elif cur_x == 1:
            events.append((proto.DPAD_RIGHT, proto.ACTION_DOWN))

    if prev_y != cur_y:
        if prev_y == 1:
            events.append((proto.DPAD_UP, proto.ACTION_UP))
        elif prev_y == -1:
            events.append((proto.DPAD_DOWN, proto.ACTION_UP))
        if cur_y == 1:
            events.append((proto.DPAD_UP, proto.ACTION_DOWN))
        elif cur_y == -1:
            events.append((proto.DPAD_DOWN, proto.ACTION_DOWN))

    return events


def process_incoming_lines(buf: bytes, chunk: bytes) -> Tuple[bytes, List[dict]]:
    """受信バッファに chunk を追加し、完成した行を decode_line してリストで返す。

    壊れた行は無視する(decode_line が None を返す行はスキップ)。戻り値は
    (残りの未完成バッファ, メッセージ一覧)。
    """
    buf += chunk
    messages: List[dict] = []
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        msg = proto.decode_line(line.decode("utf-8", errors="replace"))
        if msg is not None:
            messages.append(msg)
    return buf, messages


def apply_rumble_messages(joystick, messages: List[dict], now: float) -> Optional[float]:
    """messages 中の rumble 通知を処理し、コントローラを振動させる。

    複数あれば最後のものを採用する。振動を止めるべき時刻(time.monotonic()
    と同じ基準)を返す(rumble通知が無ければ None)。duck-typed な
    joystick(rumble(low, high, duration_ms)を持てば何でもよい)を受け取る。
    """
    stop_at: Optional[float] = None
    for msg in messages:
        if msg.get("type") != proto.MSG_TYPE_RUMBLE:
            continue
        duration_ms = msg.get("duration_ms", 0)
        joystick.rumble(1.0, 1.0, int(duration_ms))
        stop_at = now + duration_ms / 1000.0
    return stop_at
