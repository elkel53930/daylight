#!/usr/bin/env python3
"""
camera_test.py — CSIカメラから1フレーム取得して OLED (96x64) に表示する。

使い方:
    python3 camera_test.py [--loop] [--interval 0.1]

オプション:
    --loop              繰り返し表示（Ctrl+C で停止）
    --interval SECONDS  ループ間隔（秒）。デフォルト 0.1
"""

import argparse
import sys
import time
from pathlib import Path

from picamera2 import Picamera2
from PIL import Image

sys.path.insert(0, str(Path(__file__).parent / "ui"))
from ui_client import UIClient

DISPLAY_WIDTH = 96
DISPLAY_HEIGHT = 64


def capture_frame(cam: Picamera2) -> Image.Image:
    """カメラから1フレームを取得し、OLED サイズの PIL RGB 画像を返す。"""
    array = cam.capture_array("main")
    img = Image.fromarray(array[:, :, ::-1]).convert("RGB").resize(
        (DISPLAY_WIDTH, DISPLAY_HEIGHT), Image.LANCZOS
    )
    return img


def main() -> None:
    parser = argparse.ArgumentParser(description="CSI camera → OLED display test")
    parser.add_argument("--loop", action="store_true", help="連続表示")
    parser.add_argument("--interval", type=float, default=0.1, help="ループ間隔（秒）")
    args = parser.parse_args()

    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"size": (DISPLAY_WIDTH, DISPLAY_HEIGHT), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()
    # 露出安定待ち
    time.sleep(0.5)

    with UIClient() as client:
        client.connect(priority=5)

        if args.loop:
            print("Streaming to OLED (Ctrl+C to stop) ...")
            try:
                while True:
                    img = capture_frame(cam)
                    client.display(img)
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                print()
            finally:
                client.clear()
        else:
            img = capture_frame(cam)
            client.display(img)
            print("Displayed 1 frame on OLED")

    cam.stop()


if __name__ == "__main__":
    main()
