"""ゴール座標の永続化(電源OFFでも維持).

Micromouse メニューで設定したゴール座標を JSON ファイルに保存する。
ファイルが無い/壊れている場合は既定値 (8, 8) を返す。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Tuple

GOAL_PATH = Path(__file__).resolve().parent / "config" / "goal.json"
DEFAULT_GOAL: Tuple[int, int] = (8, 8)


def load_goal() -> Tuple[int, int]:
    """保存されたゴール座標 (x, y) を返す。無ければ既定 (8,8)。"""
    try:
        data = json.loads(GOAL_PATH.read_text(encoding="utf-8"))
        x = int(data["x"])
        y = int(data["y"])
        if x < 0 or y < 0:
            return DEFAULT_GOAL
        return x, y
    except Exception:
        return DEFAULT_GOAL


def save_goal(x: int, y: int) -> None:
    """ゴール座標 (x, y) を保存する。"""
    x = max(0, int(x))
    y = max(0, int(y))
    GOAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    GOAL_PATH.write_text(
        json.dumps({"x": x, "y": y}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
