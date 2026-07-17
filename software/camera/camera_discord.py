#!/usr/bin/env python3
"""
camera_discord.py — CSIカメラで撮影した画像を Discord に投稿する。

使い方:
    python3 camera_discord.py [--message "テキスト"]

オプション:
    --message TEXT  画像と一緒に投稿するテキスト（省略可）

Webhook URL の設定（discord_ip.py と同じ方法）:
    1. 環境変数 DISCORD_WEBHOOK_URL
    2. beacon/.env ファイル
    3. beacon/config.json
"""

import io
import sys
import argparse
import time
from pathlib import Path

import requests
from picamera2 import Picamera2
from PIL import Image

# beacon の設定ローダーを流用
sys.path.insert(0, str(Path(__file__).parent.parent / "beacon"))
from discord_ip import load_webhook_url

CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480


def capture_image() -> Image.Image:
    """カメラから1フレームを取得して PIL RGB 画像を返す。"""
    cam = Picamera2()
    config = cam.create_still_configuration(
        main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()
    time.sleep(1.0)  # 露出安定待ち
    try:
        array = cam.capture_array("main")
    finally:
        cam.stop()

    return Image.fromarray(array[:, :, ::-1]).convert("RGB")


def send_image(webhook_url: str, img: Image.Image, message: str = "") -> None:
    """PIL 画像を Discord Webhook に投稿する。"""
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    buf.seek(0)

    payload = {"content": message} if message else {}
    files = {"file": ("capture.jpg", buf, "image/jpeg")}

    response = requests.post(webhook_url, data=payload, files=files, timeout=30)
    response.raise_for_status()


def main() -> None:
    parser = argparse.ArgumentParser(description="CSI camera → Discord")
    parser.add_argument("--message", default="", help="画像と一緒に投稿するテキスト")
    args = parser.parse_args()

    webhook_url = load_webhook_url()

    print("Capturing image...")
    img = capture_image()
    print(f"Captured {img.width}x{img.height} image.")

    print("Sending to Discord...")
    send_image(webhook_url, img, args.message)
    print("Done.")


if __name__ == "__main__":
    main()
