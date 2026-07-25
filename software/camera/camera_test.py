#!/usr/bin/env python3
"""
camera_test.py — CSIカメラで vision アルゴリズムを OLED (96x64) 上で検証する。

default_app のメニュー(Applications → Camera Test)からも起動できる。

操作:
    右ボタン: 終了
    左ボタン: 表示モードを切り替え(下記4種を巡回)

表示モード:
    0 MASK  : HSV による黄色マスク後の画像
    1 RATIO : 黄色領域の割合(%)を撮影画像に重ねて表示
    2 WALL  : wall.py で検出した壁上面(赤)下端エッジを線分で重ねる
    3 BALL  : ball.py で検出したボールを円で重ねる

カメラは 320x240 で撮影して各アルゴリズムを実行し、結果を 96x64 に縮小して
OLED に表示する。ui_server が起動している必要がある(手動実行時のソケット
パスは camera/README.md 参照)。

実行中はアームサーボ(Futaba, software/arm/futaba_servo.py)を論理角度0度
(前方固定)に保持する。サーボを動かすとカメラの向きが変わるため
(走行時のカメラ補正と同方針)。サーボが無い環境では警告のみで続行する。
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from picamera2 import Picamera2
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))
sys.path.insert(0, str(Path(__file__).parent.parent / "vision"))
sys.path.insert(0, str(Path(__file__).parent.parent / "arm"))

from ui_client import UIClient

from ball import detect_yellow_ball, estimate_yellow_ball
from color import make_mask, yellow_ratio
from vision_types import (
    DEFAULT_RED,
    DEFAULT_YELLOW,
    BallEstimationConfig,
    WallEdgeDetectionConfig,
)
from wall import detect_nearest_red_wall_edge

CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64

DETECT_THRESHOLD = 0.003  # ボール存在判定の黄色割合(実機で要調整)
WALL_CFG = WallEdgeDetectionConfig(seed=0)

MODE_MASK, MODE_RATIO, MODE_WALL, MODE_BALL = range(4)
MODE_LABELS = {MODE_MASK: "MASK", MODE_RATIO: "RATIO", MODE_WALL: "WALL", MODE_BALL: "BALL"}
NUM_MODES = 4


def capture_bgr(cam: Picamera2) -> np.ndarray:
    """1フレーム撮影して BGR 配列(H,W,3)を返す。

    format=RGB888 の capture_array は numpy 上では BGR 並び(camera_test 旧版・
    vision_algorithm.md の入力仕様と同じ)。
    """
    return cam.capture_array("main")


def _base_display(bgr: np.ndarray) -> Image.Image:
    """撮影画像(BGR)を表示用 RGB 96x64 に縮小した PIL 画像。"""
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    return Image.fromarray(rgb).convert("RGB").resize(
        (DISPLAY_WIDTH, DISPLAY_HEIGHT), Image.LANCZOS
    )


def render(bgr: np.ndarray, mode: int) -> Image.Image:
    """現在のモードに応じた検証用オーバーレイ画像(RGB 96x64)を作る。"""
    ch, cw = bgr.shape[:2]
    sx = DISPLAY_WIDTH / cw
    sy = DISPLAY_HEIGHT / ch

    if mode == MODE_MASK:
        mask = make_mask(bgr, DEFAULT_YELLOW)
        vis = np.zeros((ch, cw, 3), dtype=np.uint8)
        vis[mask] = (255, 255, 0)  # 黄色マスクを黄で表示
        img = Image.fromarray(vis).resize((DISPLAY_WIDTH, DISPLAY_HEIGHT), Image.NEAREST)
    else:
        img = _base_display(bgr)

    draw = ImageDraw.Draw(img)

    if mode == MODE_RATIO:
        ratio = yellow_ratio(bgr)
        draw.text((2, 12), f"{ratio * 100:.1f}%", fill=(0, 255, 0))

    elif mode == MODE_WALL:
        edge = detect_nearest_red_wall_edge(bgr, DEFAULT_RED, WALL_CFG)
        if edge is not None:
            a, b = edge
            p0 = (0.0, b * sy)
            p1 = ((cw - 1) * sx, (a * (cw - 1) + b) * sy)
            draw.line([p0, p1], fill=(255, 0, 0), width=2)
        else:
            draw.text((2, 12), "no edge", fill=(255, 0, 0))

    elif mode == MODE_BALL:
        cfg = BallEstimationConfig(seed=0, min_radius_px=6.0, max_radius_px=0.5 * cw)
        present = detect_yellow_ball(bgr, DEFAULT_YELLOW, DETECT_THRESHOLD)
        ball = estimate_yellow_ball(bgr, DEFAULT_YELLOW, cfg) if present else None
        if ball is not None:
            cx, cy = ball.center_x * sx, ball.center_y * sy
            rx, ry = (ball.diameter / 2.0) * sx, (ball.diameter / 2.0) * sy
            draw.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], outline=(255, 0, 0), width=2)
            draw.line([cx - 3, cy, cx + 3, cy], fill=(255, 0, 0))
            draw.line([cx, cy - 3, cx, cy + 3], fill=(255, 0, 0))
        else:
            draw.text((2, 12), "no ball", fill=(255, 0, 0))

    # モードラベル(左上)
    draw.text((2, 2), MODE_LABELS[mode], fill=(255, 255, 255))
    return img


def _rising(prev: dict, cur: dict, key: str) -> bool:
    """released → pressed/long_pressed の立ち上がりを検出する。"""
    return prev.get(key) == "released" and cur.get(key) in ("pressed", "long_pressed")


def _fix_arm_servo_forward():
    """アームサーボ(Futaba)を論理角度0度(前方固定)に保持する。

    カメラは Futaba サーボを動かすと向きが変わるため、検証中は前方固定にする
    (走行時のカメラ補正と同方針)。生成に失敗しても検証は続行できるよう
    best-effort(サーボ無し環境では警告のみ)。戻り値は FutabaServo または None。
    """
    try:
        from futaba_servo import FutabaServo

        servo = FutabaServo()
        servo.set_torque(True)
        servo.set_angle(0.0)
        return servo
    except Exception as e:
        print(f"# アームサーボを0度固定できません(続行します): {e}")
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description="CSI camera → vision 検証(OLED)")
    parser.add_argument("--interval", type=float, default=0.05, help="ループ間隔(秒)")
    args = parser.parse_args()

    # カメラ向きを固定するためアームサーボを0度(前方)に保持
    servo = _fix_arm_servo_forward()

    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()
    time.sleep(0.5)  # 露出安定待ち

    mode = MODE_MASK
    prev_buttons = {"left": "released", "right": "released"}

    try:
        with UIClient() as client:
            client.connect(priority=5)
            try:
                while True:
                    buttons = client.get_buttons()
                    if _rising(prev_buttons, buttons, "right"):
                        break
                    if _rising(prev_buttons, buttons, "left"):
                        mode = (mode + 1) % NUM_MODES
                    prev_buttons = buttons

                    bgr = capture_bgr(cam)
                    client.display(render(bgr, mode))
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                pass
            finally:
                client.clear()
    finally:
        cam.stop()
        if servo is not None:
            servo.close()  # トルクオフしてシリアルを閉じる


if __name__ == "__main__":
    main()
