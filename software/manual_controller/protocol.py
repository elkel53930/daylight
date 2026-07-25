"""機体(ラズパイ)⇔操縦PC間の通信プロトコル定義。

依存無しの純 Python(json 標準ライブラリのみ)。robot 側(remote_server.py)
と PC 側(remote_client.py)の両方から import される共有定義であり、
ネットワーク・ゲームコントローラ・シリアルいずれにも依存しないため
そのままユニットテストできる。

## メッセージ形式

TCP 接続上で改行区切りの JSON を1メッセージ1行として送る(単純さ優先、
Content-Length 等のフレーミングは行わない。1メッセージが十分小さいため
改行区切りで実用上問題ない)。

    {"type": "button", "name": "dpad_up", "action": "down"}\n
    {"type": "heartbeat"}\n

- type="button": ボタンの押下/解放イベント。name は BUTTON_NAMES のいずれか、
  action は "down"(押下) または "up"(解放)。
- type="heartbeat": PC 側が接続維持を示すために一定間隔で送る(実際の
  ボタン入力が無くても送る)。robot 側はこれが一定時間途絶えたらリンク
  切断とみなし緊急停止する(remote_server.py の WATCHDOG_TIMEOUT_S)。

## zeroconf

robot 側はこの SERVICE_TYPE で自身の TCP サーバーをアドバタイズし、PC 側は
同じ SERVICE_TYPE で検索する。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

# --- zeroconf ---
SERVICE_TYPE = "_daylight-remote._tcp.local."
DEFAULT_CONTROL_PORT = 50123

# --- ボタン名(PC 側の物理ボタンとの対応は remote_client.py / input_mapping.py) ---
DPAD_UP = "dpad_up"
DPAD_DOWN = "dpad_down"
DPAD_LEFT = "dpad_left"
DPAD_RIGHT = "dpad_right"
TRIANGLE = "triangle"
CIRCLE = "circle"
CROSS = "cross"
SQUARE = "square"
L1 = "l1"
R1 = "r1"

BUTTON_NAMES = frozenset(
    {DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, TRIANGLE, CIRCLE, CROSS, SQUARE, L1, R1}
)

ACTION_DOWN = "down"
ACTION_UP = "up"
ACTIONS = frozenset({ACTION_DOWN, ACTION_UP})

MSG_TYPE_BUTTON = "button"
MSG_TYPE_HEARTBEAT = "heartbeat"


@dataclass(frozen=True)
class ButtonEvent:
    name: str
    action: str  # ACTION_DOWN / ACTION_UP


def encode_button_event(name: str, action: str) -> bytes:
    """ボタンイベントを1行分の JSON バイト列(末尾 \\n 付き)にする。"""
    if name not in BUTTON_NAMES:
        raise ValueError(f"unknown button name: {name}")
    if action not in ACTIONS:
        raise ValueError(f"unknown action: {action}")
    line = json.dumps({"type": MSG_TYPE_BUTTON, "name": name, "action": action})
    return (line + "\n").encode("utf-8")


def encode_heartbeat() -> bytes:
    """ハートビートメッセージを1行分の JSON バイト列にする。"""
    return (json.dumps({"type": MSG_TYPE_HEARTBEAT}) + "\n").encode("utf-8")


def decode_line(line: str) -> Optional[dict]:
    """受信した1行を JSON としてパースする。

    不正な行(壊れた JSON・型不正)は None を返す(呼び出し側は無視して
    次の行を待てばよい。ネットワーク越しの1行破損で通信全体を落とさない)。
    """
    line = line.strip()
    if not line:
        return None
    try:
        data = json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None

    msg_type = data.get("type")
    if msg_type == MSG_TYPE_HEARTBEAT:
        return {"type": MSG_TYPE_HEARTBEAT}
    if msg_type == MSG_TYPE_BUTTON:
        name = data.get("name")
        action = data.get("action")
        if name not in BUTTON_NAMES or action not in ACTIONS:
            return None
        return {"type": MSG_TYPE_BUTTON, "name": name, "action": action}
    return None
