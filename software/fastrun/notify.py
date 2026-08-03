"""notify.py — 走行前のブザー通知(2026-08-03 ユーザー指示)。

ロボットを動かすときは、動かす1秒ほど前にブザーを鳴らして周りに知らせる。
ブザーは ui_server 経由で鳴らす(GPIO13を直接 claim すると ALT0 が外れて
無音になるため。CLAUDE.md 参照)。ui_server は default_app が priority=100 で
保持しているので、一時的に高優先度で接続→再生→切断して戻す。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path


def beep(melody: str = "ee") -> None:
    """ui_server 経由で短いブザーを鳴らす(失敗しても例外を握りつぶす)。"""
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ui"))
    try:
        from ui_client import UIClient
    except Exception:
        return
    c = UIClient()
    try:
        c.connect(priority=3)
        c.play(melody)
    except Exception:
        pass
    finally:
        try:
            c.disconnect()
        except Exception:
            pass


def warn_before_move(delay_s: float = 1.0, melody: str = "ee") -> None:
    """ブザーを鳴らし、動作開始まで delay_s 待つ。走行の直前に必ず呼ぶ。"""
    beep(melody)
    time.sleep(delay_s)


if __name__ == "__main__":
    warn_before_move()
    print("beeped")
