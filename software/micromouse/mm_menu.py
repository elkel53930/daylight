#!/usr/bin/env python3
"""Micromouse メニューアプリ(default_app の Applications から起動).

サブメニュー:
  - Set Goal : ゴール座標(x,y)を設定。設定値は JSON に保存され電源OFFでも維持。
  - Start Run: 保存されたゴール座標で探索走行を開始。

操作:
  L 短押し: 項目/桁の移動
  R 短押し: 決定 / 値のインクリメント
  R 長押し: メニューでは終了、ゴール編集ではデクリメント
  L 長押し: ゴール編集をキャンセルして戻る
"""

from __future__ import annotations

import sys
import time
import types
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "ui"))

from PIL import Image, ImageDraw  # noqa: E402

from ui_client import UIClient  # noqa: E402
from goal_store import load_goal, save_goal  # noqa: E402
from config import MicromouseConfig  # noqa: E402
from micromouse_app import OledUI, run_mission, MELODY_START, MELODY_FINISH, MELODY_ERROR  # noqa: E402

W, H = 96, 64
GOAL_MAX = 15  # 16x16 迷路


def _rising(prev: dict, cur: dict, key: str, state: str) -> bool:
    return prev.get(key) != state and cur.get(key) == state


def _short_released(prev: dict, cur: dict, key: str) -> bool:
    """短押し確定: pressed のまま released に戻った瞬間(長押しを除外)。"""
    return prev.get(key) == "pressed" and cur.get(key) == "released"


def _img():
    return Image.new("RGB", (W, H), "black")


def draw_main(selected: int, goal, note: str = "") -> Image.Image:
    img = _img()
    d = ImageDraw.Draw(img)
    d.text((2, 0), "MICROMOUSE", fill="white")
    top = note if note else f"Goal ({goal[0]},{goal[1]})"
    d.text((2, 10), top[:16], fill="white")
    items = ["Set Goal", "Start Run", "Reset Maze", "Quit"]
    for i, it in enumerate(items):
        prefix = ">" if i == selected else " "
        d.text((2, 22 + i * 10), f"{prefix}{it}", fill="white")
    return img


def draw_goal(sel: int, x: int, y: int) -> Image.Image:
    img = _img()
    d = ImageDraw.Draw(img)
    d.text((2, 1), "SET GOAL", fill="white")

    def row(i, label, val):
        prefix = ">" if sel == i else " "
        return f"{prefix}{label}: {val:>2}"

    d.text((2, 15), row(0, "X", x), fill="white")
    d.text((2, 27), row(1, "Y", y), fill="white")
    prefix = ">" if sel == 2 else " "
    d.text((2, 43), f"{prefix}[ Save ]", fill="white")
    d.text((2, 54), "L:next R:+ Rh:-", fill="white")
    return img


def edit_goal(client: UIClient, goal) -> tuple:
    x, y = goal
    sel = 0
    prev = {"left": "released", "right": "released"}
    while True:
        b = client.get_buttons()
        if _rising(prev, b, "right", "long_pressed"):
            if sel == 0:
                x = (x - 1) % (GOAL_MAX + 1)
            elif sel == 1:
                y = (y - 1) % (GOAL_MAX + 1)
        elif _rising(prev, b, "left", "long_pressed"):
            prev = b
            return goal  # キャンセル
        elif _short_released(prev, b, "right"):
            if sel == 0:
                x = (x + 1) % (GOAL_MAX + 1)
            elif sel == 1:
                y = (y + 1) % (GOAL_MAX + 1)
            else:
                save_goal(x, y)
                try:
                    client.play("ceg")
                except Exception:
                    pass
                prev = b
                return (x, y)
        elif _short_released(prev, b, "left"):
            sel = (sel + 1) % 3
        prev = b
        client.display(draw_goal(sel, x, y))
        time.sleep(0.05)


def _safe_stop_motors() -> None:
    """念のための最終停止(MOT,0,0)。走行終了後にシリアルを開き直して送る。"""
    try:
        import serial

        s = serial.Serial("/dev/ttyUSB0", 3000000, timeout=0.2)
        try:
            s.write(b"MOT,0,0\n")
            s.flush()
            time.sleep(0.1)
        finally:
            s.close()
    except Exception:
        pass


def start_run(client: UIClient, maze):
    """保存済みゴール座標で探索走行を実行する.

    戻り値: (最終状態の短い文字列, 更新後の迷路オブジェクト)。
    迷路はアプリ終了まで保持される(ミッション間で共有)。
    """
    config = MicromouseConfig.load()
    gx, gy = load_goal()
    config.goal_x_min = config.goal_x_max = gx
    config.goal_y_min = config.goal_y_max = gy

    if maze is None:
        from maze import Maze

        maze = Maze(config.maze_size)

    args = types.SimpleNamespace(sim=None, port=config.port, baud=None)

    # メニューのUIを一旦手放して、走行用UI(同一優先度)に切り替える
    try:
        client.clear()
    except Exception:
        pass
    client.disconnect()

    ui = None
    result = "----"
    try:
        ui = OledUI(priority=5)
        # 安全: ブザー→2秒待ってから動かす。走行中はLで強制停止できる旨を表示。
        try:
            ui.client.play(MELODY_START)
        except Exception:
            pass
        try:
            ui.show_message("MICROMOUSE", "starting...", "", "L:Stop run")
        except Exception:
            pass
        time.sleep(2.0)
        final, _err = run_mission(args, config, ui, maze=maze)
        result = final.value
    except Exception as e:  # UI切断・走行例外でもメニューへ戻す
        print(f"run error: {e}")
        result = "ERROR"
    finally:
        try:
            if ui is not None:
                ui.close()
        except Exception:
            pass
        # 走行終了/中断時にモーター停止を確実にする
        _safe_stop_motors()
        # メニューUIへ復帰(アプリは継続。終了はメニューのQuit/長押し)
        client.connect(priority=5)

    try:
        client.play(MELODY_FINISH if result == "FINISHED" else MELODY_ERROR)
    except Exception:
        pass
    return result, maze


def menu_loop(client: UIClient) -> None:
    goal = load_goal()
    selected = 0
    note = ""
    maze_state = None  # アプリ終了まで保持(Reset Mazeでクリア)
    prev = {"left": "released", "right": "released"}
    while True:
        b = client.get_buttons()
        # メインメニューではL長押し/R長押しのどちらでも終了(default_appへ戻る)
        if _rising(prev, b, "right", "long_pressed") or _rising(
            prev, b, "left", "long_pressed"
        ):
            prev = b
            return
        if _short_released(prev, b, "right"):
            prev = b
            if selected == 0:
                goal = edit_goal(client, goal)
                note = ""
            elif selected == 1:
                note = "running..."
                client.display(draw_main(selected, goal, note))
                note, maze_state = start_run(client, maze_state)
            elif selected == 2:
                maze_state = None
                note = "maze reset"
                try:
                    client.play("c")
                except Exception:
                    pass
            else:
                return  # Quit
            continue
        if _short_released(prev, b, "left"):
            selected = (selected + 1) % 4
        prev = b
        client.display(draw_main(selected, goal, note))
        time.sleep(0.05)


def main() -> int:
    try:
        with UIClient() as client:
            client.connect(priority=5)
            try:
                menu_loop(client)
            finally:
                try:
                    client.clear()
                except Exception:
                    pass
    except ConnectionError as e:
        print(f"UI接続失敗: {e}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
