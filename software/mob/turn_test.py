#!/usr/bin/env python3
"""
turn_test.py — mob の HOLD(その場静止)・TURN(その場旋回)を実機で
OLED越しに試すためのテストツール。

default_app のメニュー(Applications → Turn Test)からも起動できる。

操作:
    左ボタン: メニュー選択を送る(HOLD/TURN実行中は無視)
    右ボタン(短押し): メニューでは選択中の動作を実行、実行中ならMOT,0,0で
                       停止してメニューに戻る
    右ボタン(長押し): MOT,0,0で停止して終了

起動時に自動でジャイロキャリブレーション(GCAL)を行う。機体は静止させて
おくこと。
"""

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Optional

import serial
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))
from ui_client import UIClient  # noqa: E402

DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64

# (表示ラベル, mobへ送るコマンド)。HOLD/TURNは送りっぱなしコマンド
# (継続動作、DONE無し)のため実行後は監視画面へ遷移する。GCALはDONEを
# 待つ単発コマンドとして扱う。
ACTIONS = [
    ("HOLD", "HOLD"),
    ("TURN +90", "TURN,1.5708"),
    ("TURN -90", "TURN,-1.5708"),
    ("TURN +180", "TURN,3.1416"),
    ("TURN -180", "TURN,-3.1416"),
    ("GCAL", "GCAL"),
]


class MobLink:
    """mobとの簡易シリアル通信(このツール専用)。"""

    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def send(self, cmd: str) -> None:
        self.ser.write((cmd + "\n").encode("ascii"))

    def _readline(self, timeout_s: float) -> Optional[str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if raw:
                return raw.decode("ascii", errors="replace").strip()
        return None

    def wait_for(self, prefix: str, timeout_s: float) -> Optional[str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self._readline(timeout_s=max(0.0, deadline - time.monotonic()))
            if line is None:
                break
            if line.startswith(prefix):
                return line
        return None

    def read_sen(self) -> Optional[dict]:
        self.ser.reset_input_buffer()
        self.send("SEN")
        line = self.wait_for("SEN,", timeout_s=0.5)
        if line is None:
            return None
        parts = line.split(",")
        if len(parts) != 13:
            return None
        try:
            return {
                "vbatt": float(parts[2]),
                "odo_dist": float(parts[9]),
                "odo_ang": float(parts[10]),
            }
        except ValueError:
            return None


def _rising(prev: dict, cur: dict, key: str, state: str) -> bool:
    return prev.get(key) != state and cur.get(key) == state


def draw_menu(selected: int, status: str) -> Image.Image:
    img = Image.new("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    draw.text((2, 0), "Turn Test", fill=(255, 255, 255))
    y = 10
    for i, (label, _) in enumerate(ACTIONS):
        prefix = ">" if i == selected else " "
        color = (0, 255, 0) if i == selected else (200, 200, 200)
        draw.text((2, y), f"{prefix}{label}", fill=color)
        y += 8
    draw.text((2, DISPLAY_HEIGHT - 8), status[:20], fill=(255, 255, 0))
    return img


def draw_running(label: str, sen: Optional[dict], elapsed_s: float) -> Image.Image:
    img = Image.new("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    draw.text((2, 0), f"RUN: {label}", fill=(0, 255, 0))
    if sen is not None:
        deg = math.degrees(sen["odo_ang"])
        draw.text((2, 14), f"ang: {deg:+.1f}deg", fill=(255, 255, 255))
        draw.text((2, 24), f"dist: {sen['odo_dist']:+.1f}mm", fill=(255, 255, 255))
        draw.text((2, 34), f"vbatt: {sen['vbatt']:.2f}V", fill=(255, 255, 255))
    else:
        draw.text((2, 14), "SEN: (no resp)", fill=(255, 0, 0))
    draw.text((2, 44), f"t={elapsed_s:.1f}s", fill=(200, 200, 200))
    draw.text((2, DISPLAY_HEIGHT - 16), "R short: stop", fill=(150, 150, 150))
    draw.text((2, DISPLAY_HEIGHT - 8), "R long: quit", fill=(150, 150, 150))
    return img


def calibrate(link: MobLink, client: UIClient) -> None:
    client.display(draw_menu(0, "Calibrating..."))
    link.send("GCAL")
    link.wait_for("DONE", timeout_s=3.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="mobのシリアルポート")
    parser.add_argument("--baud", type=int, default=3000000)
    args = parser.parse_args()

    link = MobLink(args.port, args.baud)

    try:
        with UIClient() as client:
            client.connect(priority=5)
            try:
                calibrate(link, client)

                selected = 0
                running_cmd: Optional[str] = None
                running_label = ""
                run_start = 0.0
                prev_buttons = {"left": "released", "right": "released"}
                last_sen: Optional[dict] = None
                last_sen_t = 0.0

                while True:
                    buttons = client.get_buttons()

                    if running_cmd is None:
                        if _rising(prev_buttons, buttons, "right", "long_pressed"):
                            prev_buttons = buttons
                            break
                        if _rising(prev_buttons, buttons, "right", "pressed"):
                            label, cmd = ACTIONS[selected]
                            if cmd == "GCAL":
                                prev_buttons = buttons
                                calibrate(link, client)
                                continue
                            link.send(cmd)
                            running_cmd = cmd
                            running_label = label
                            run_start = time.monotonic()
                            last_sen = None
                            last_sen_t = 0.0
                        elif _rising(prev_buttons, buttons, "left", "pressed"):
                            selected = (selected + 1) % len(ACTIONS)
                        prev_buttons = buttons

                        client.display(draw_menu(selected, "L:select R:run R-long:quit"))
                    else:
                        if _rising(prev_buttons, buttons, "right", "long_pressed"):
                            link.send("MOT,0,0")
                            prev_buttons = buttons
                            break
                        if _rising(prev_buttons, buttons, "right", "pressed"):
                            link.send("MOT,0,0")
                            running_cmd = None
                        prev_buttons = buttons

                        now = time.monotonic()
                        if now - last_sen_t >= 0.2:
                            last_sen = link.read_sen()
                            last_sen_t = now
                        client.display(draw_running(running_label, last_sen, now - run_start))

                    time.sleep(0.05)
            finally:
                link.send("MOT,0,0")
                client.clear()
    finally:
        link.close()


if __name__ == "__main__":
    main()
