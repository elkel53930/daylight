#!/usr/bin/env python3
"""
motion_test.py — mob の HOLD(その場静止)・TURN(その場旋回)・PATTERN
(仮想ターゲット追従のパス走行)を実機でOLED越しに試すためのテストツール。
その場旋回だけでなく並進を伴う走行も試せるため turn_test.py から改名した
(2026-08-02)。

PATTERNの走行パスはmob.ino側にハードコードされておらず、pattern.pyの
DEFAULT_TEST_PATTERNをPCLEAR/PADD/PRUNコマンドで実行時にmobへ送信する
(2026-08-02、走行パターンをPC側から指定できるように変更)。

default_app のメニュー(Applications → Motion Test)からも起動できる。

操作:
    左ボタン: メニュー選択を送る(HOLD/TURN実行中は無視)
    右ボタン(短押し): メニューでは選択中の動作を実行、実行中ならMOT,0,0で
                       停止してメニューに戻る
    右ボタン(長押し): MOT,0,0で停止して終了

HOLD/TURNを選んで右ボタンを押すと、ビープ音を鳴らして1秒待ってから
実行する(押した指が機体に当たらないよう離れる時間を作るため)。

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
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))
from ui_client import UIClient  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent.parent / "fastrun"))
from camera_align import OnboardCamera  # noqa: E402
from dev_maze import build_dev_maze  # noqa: E402
from geometry import Direction  # noqa: E402
from liner_center import recenter_cell  # noqa: E402
from liner_pose import LinerPose, direction_to_gyro_deg  # noqa: E402
import recenter as recenter_ops  # noqa: E402

from pattern import DEFAULT_TEST_PATTERN, send_pattern  # noqa: E402

DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64

# default_app/renderer.py と同じフォント・行送り規約(96x64 OLED向けに
# 実機で確認済みの値)。PILの無指定デフォルトフォントは行高が読みにくく
# 重なりやすいため使わない。
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_SIZE = 8
LINE_HEIGHT = 8
try:
    FONT = ImageFont.truetype(FONT_PATH, FONT_SIZE)
except Exception:
    FONT = ImageFont.load_default()

# (表示ラベル, mobへ送るコマンド)。HOLD/TURNは送りっぱなしコマンド
# (継続動作、DONE無し)のため実行後は監視画面へ遷移する。GCALはDONEを
# 待つ単発コマンドとして扱う。PATTERNはコマンド文字列ではなくPCLEAR/PADD/
# PRUNの複数コマンド送信が必要なため特殊値 "PATTERN" で表し、main()側で
# 個別分岐する(2026-08-02、mob.ino側のハードコード撤去に伴いpattern.py
# 経由の送信に変更)。
ACTIONS = [
    ("HOLD", "HOLD"),
    ("TURN +90", "TURN,1.5708"),
    ("TURN -90", "TURN,-1.5708"),
    ("TURN +180", "TURN,3.1416"),
    ("TURN -180", "TURN,-3.1416"),
    ("PATTERN", "PATTERN"),
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

    def stop(self) -> None:
        self.send("MOT,0,0")

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


def _short_press_released(prev: dict, cur: dict, key: str) -> bool:
    """短押し確定の検出: 長押し閾値に達する前にreleasedへ戻った瞬間。

    ui_server の状態機械は長押し中も必ず一旦 "pressed" を経由するため
    (released → pressed → 一定時間後にlong_pressedへ昇格)、押した瞬間の
    "pressed" への立ち上がりだけを見ると長押しの最中にも短押しイベントが
    誤発火する(2026-08-02、実機でユーザーが確認)。"pressed" のまま
    releasedに戻ったときだけ短押しとして扱う(long_pressedを経由していれば
    prevは"long_pressed"になっているのでここでは発火しない)。
    """
    return prev.get(key) == "pressed" and cur.get(key) == "released"


def _blank() -> Image.Image:
    return Image.new("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), (0, 0, 0))


def draw_menu(selected: int) -> Image.Image:
    # 項目(len(ACTIONS)行) + フッター(1行) をLINE_HEIGHT=8pxで詰めて
    # 96x64のOLEDにちょうど収める(2026-08-02: 以前は行が多すぎて
    # 文字が重なっていた)。
    img = _blank()
    draw = ImageDraw.Draw(img)
    y = 0
    for i, (label, _) in enumerate(ACTIONS):
        prefix = ">" if i == selected else " "
        color = (0, 255, 0) if i == selected else (200, 200, 200)
        draw.text((0, y), f"{prefix}{label}", fill=color, font=FONT)
        y += LINE_HEIGHT
    draw.text((0, y), "R-long: Quit", fill=(255, 255, 0), font=FONT)
    return img


def draw_running(label: str, sen: Optional[dict], elapsed_s: float) -> Image.Image:
    img = _blank()
    draw = ImageDraw.Draw(img)
    if sen is not None:
        deg = math.degrees(sen["odo_ang"])
        ang_text = f"{deg:+.1f}deg"
        dist_text = f"{sen['odo_dist']:+.1f}mm"
        vbatt_text = f"{sen['vbatt']:.2f}V"
    else:
        ang_text = dist_text = vbatt_text = "(no SEN resp)"

    lines = [
        (f"RUN: {label}", (0, 255, 0)),
        (f"ang:  {ang_text}", (255, 255, 255)),
        (f"dist: {dist_text}", (255, 255, 255)),
        (f"batt: {vbatt_text}", (255, 255, 255)),
        (f"t={elapsed_s:.1f}s", (200, 200, 200)),
        ("R-short: Stop", (255, 255, 0)),
        ("R-long: Quit", (255, 255, 0)),
    ]
    y = 0
    for text, color in lines:
        draw.text((0, y), text, fill=color, font=FONT)
        y += LINE_HEIGHT
    return img


def draw_countdown(label: str) -> Image.Image:
    img = _blank()
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), "Starting:", fill=(255, 255, 0), font=FONT)
    draw.text((0, LINE_HEIGHT), label, fill=(0, 255, 0), font=FONT)
    draw.text((0, LINE_HEIGHT * 3), "Hands off!", fill=(255, 0, 0), font=FONT)
    return img


def draw_status(title: str, detail: str) -> Image.Image:
    img = _blank()
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), title, fill=(255, 255, 0), font=FONT)
    draw.text((0, LINE_HEIGHT), detail, fill=(200, 200, 200), font=FONT)
    return img


def calibrate(link: MobLink, client: UIClient) -> None:
    img = _blank()
    ImageDraw.Draw(img).text((0, 0), "Calibrating...", fill=(255, 255, 0), font=FONT)
    client.display(img)
    link.send("GCAL")
    link.wait_for("DONE", timeout_s=3.0)


def recenter_before_pattern(link: MobLink, client: UIClient) -> bool:
    """PATTERN開始前に、既知の初期姿勢(0,0,N)を前提に壁上面補正する。"""
    maze = build_dev_maze()
    pose = LinerPose(0, 0, Direction.N)
    client.display(draw_status("Recentering", "cell=(0,0) dir=N"))
    with OnboardCamera() as cam:
        init_heading_deg = direction_to_gyro_deg(Direction.N)
        link.stop()
        time.sleep(0.15)
        link.send(f"SANG,{math.radians(init_heading_deg):.5f}")
        if link.wait_for("DONE", timeout_s=1.0) is None:
            client.display(draw_status("Recenter NG", "SANG timeout"))
            time.sleep(1.0)
            return False
        result = recenter_cell(link, cam, maze, pose)
        link.stop()
        time.sleep(0.15)

    y_ok = result.get("y") is not None and result["y"].ok
    x_ok = result.get("x") is not None and result["x"].ok
    if not (y_ok and x_ok):
        client.display(draw_status("Recenter NG", f"y={y_ok} x={x_ok}"))
        time.sleep(1.0)
        return False

    # 壁上面補正は最後にX軸面(E/W)で終わるため、PATTERN開始前に北へ向き直す。
    north_deg = direction_to_gyro_deg(Direction.N)
    recenter_ops.turn_to(link, north_deg)
    link.stop()
    time.sleep(0.15)
    link.send(f"SANG,{math.radians(north_deg):.5f}")
    if link.wait_for("DONE", timeout_s=1.0) is None:
        client.display(draw_status("Recenter NG", "north sync timeout"))
        time.sleep(1.0)
        return False

    client.display(draw_status("Recenter OK", "start pattern"))
    time.sleep(0.5)
    return True


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
                        if _short_press_released(prev_buttons, buttons, "right"):
                            label, cmd = ACTIONS[selected]
                            prev_buttons = buttons
                            if cmd == "GCAL":
                                calibrate(link, client)
                                continue
                            # 押した瞬間に動き出すとボタンを押している指が
                            # 機体に当たるため、ビープ音+1秒待ってから実行する
                            # (2026-08-02追加、ユーザー指摘)。
                            client.play("c")
                            client.display(draw_countdown(label))
                            time.sleep(1.0)
                            if cmd == "PATTERN":
                                if not recenter_before_pattern(link, client):
                                    running_cmd = None
                                    continue
                                send_pattern(link, DEFAULT_TEST_PATTERN)
                            else:
                                link.send(cmd)
                            running_cmd = cmd
                            running_label = label
                            run_start = time.monotonic()
                            last_sen = None
                            last_sen_t = 0.0
                            continue
                        elif _rising(prev_buttons, buttons, "left", "pressed"):
                            selected = (selected + 1) % len(ACTIONS)
                        prev_buttons = buttons

                        client.display(draw_menu(selected))
                    else:
                        if _rising(prev_buttons, buttons, "right", "long_pressed"):
                            link.send("MOT,0,0")
                            prev_buttons = buttons
                            break
                        if _short_press_released(prev_buttons, buttons, "right"):
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
