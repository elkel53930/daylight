#!/usr/bin/env python3
"""vision パッケージの実機動作確認用 HTTP ビューア。

Picamera2 で撮影した画像に対し
  - detect_yellow_ball        (ボール検出の有無)
  - estimate_yellow_ball      (中心・直径・自信度)
  - detect_nearest_red_wall_edge (最も手前の赤壁下端エッジ)
を実行し、結果を元画像に重ねて描画した JPEG を HTTP で配信する。

    software/venv/bin/python3 software/vision/vision_http_test.py
    → ブラウザで http://<ラズパイのIP>:8080/ を開く

オーバーレイ内容:
  - 左上に "Ball: YES/NO" と自信度(conf=…)
  - 推定したボールを赤い丸(中心 + 円周)で描画
  - 検出した赤エッジを線分で描画(画像左端〜右端に伸ばす)

依存は picamera2 / Pillow / numpy / cv2(color.py が使用)。HTTP は標準
ライブラリ http.server を使う(flask 不要)。flask を使いたい場合はこの
ファイルの `build_overlay_jpeg()` をそのまま呼べば移植できる。
"""

from __future__ import annotations

import argparse
import io
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).parent))

from ball import detect_yellow_ball, estimate_yellow_ball
from vision_types import (
    BallEstimationConfig,
    DEFAULT_RED,
    DEFAULT_YELLOW,
    WallEdgeDetectionConfig,
)
from wall import detect_nearest_red_wall_edge

DETECT_THRESHOLD = 0.003  # 黄色ピクセル割合(実機で要調整)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
WALL_CFG = WallEdgeDetectionConfig(seed=0)


def build_overlay_jpeg(bgr: np.ndarray, quality: int = 80) -> bytes:
    """BGR 画像に検出結果を重ねた JPEG バイト列を返す。"""
    h, w = bgr.shape[:2]
    # 半径上限を画像幅基準で設定(遮蔽で欠けたボールへの過大な円フィットを
    # 弾く。Twilight の max_ball_radius ゲートと同趣旨)。
    ball_cfg = BallEstimationConfig(seed=0, min_radius_px=6.0, max_radius_px=0.5 * w)
    present = detect_yellow_ball(bgr, DEFAULT_YELLOW, DETECT_THRESHOLD)
    ball = estimate_yellow_ball(bgr, DEFAULT_YELLOW, ball_cfg) if present else None
    edge = detect_nearest_red_wall_edge(bgr, DEFAULT_RED, WALL_CFG)

    rgb = bgr[:, :, ::-1]  # BGR → RGB(表示用)
    img = Image.fromarray(np.ascontiguousarray(rgb)).convert("RGB")
    draw = ImageDraw.Draw(img)

    # ボール検出の有無・自信度
    status = f"Ball: {'YES' if present else 'NO'}"
    if ball is not None:
        status += f"  conf={ball.confidence:.2f}"
    draw.text((4, 4), status, fill=GREEN)

    # 推定したボール(赤い丸 + 中心)
    if ball is not None:
        r = ball.diameter / 2.0
        cx, cy = ball.center_x, ball.center_y
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=RED, width=3)
        draw.line([cx - 5, cy, cx + 5, cy], fill=RED, width=2)
        draw.line([cx, cy - 5, cx, cy + 5], fill=RED, width=2)
        draw.text((4, 18), f"d={ball.diameter:.0f}px ({cx:.0f},{cy:.0f})", fill=RED)

    # 赤エッジ(y = a*x + b を画像左端〜右端の線分で)
    if edge is not None:
        a, b = edge
        p0 = (0.0, b)
        p1 = (float(w - 1), a * (w - 1) + b)
        draw.line([p0, p1], fill=(255, 255, 0), width=3)
        draw.text((4, h - 16), f"edge y={a:.3f}x+{b:.0f}", fill=(255, 255, 0))
    else:
        draw.text((4, h - 16), "edge: none", fill=(255, 255, 0))

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class CameraSource:
    """Picamera2 を 1 つ起動して使い回す(スレッド安全のためロックで保護)。"""

    def __init__(self, width: int, height: int):
        from picamera2 import Picamera2

        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._cam.configure(cfg)
        self._cam.start()
        time.sleep(0.5)  # 露出安定待ち
        self._lock = threading.Lock()

    def capture_bgr(self) -> np.ndarray:
        # format=RGB888 の capture_array は numpy 上では BGR 並び
        # (camera_test.py が [:, :, ::-1] で RGB へ直しているのと同じ)。
        # vision_algorithm.md の入力仕様(BGR)にそのまま合致する。
        with self._lock:
            return self._cam.capture_array("main")

    def close(self) -> None:
        try:
            self._cam.stop()
        except Exception:
            pass


PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<title>vision test</title></head><body style="margin:0;background:#111">
<img id="v" style="width:100%;max-width:960px;display:block;margin:auto">
<script>
function tick(){document.getElementById('v').src='/frame.jpg?'+Date.now();}
setInterval(tick,300);tick();
</script></body></html>"""


def make_handler(source: CameraSource):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # アクセスログを抑制
            pass

        def do_GET(self):
            if self.path.startswith("/frame.jpg"):
                try:
                    bgr = source.capture_bgr()
                    jpeg = build_overlay_jpeg(bgr)
                except Exception as e:  # 撮影・処理失敗でもサーバは落とさない
                    self.send_error(500, str(e))
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGE)

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description="vision パッケージ HTTP ビューア")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    source = CameraSource(args.width, args.height)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(source))
    print(f"http://{args.host}:{args.port}/  (Ctrl+C で終了)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.shutdown()
        source.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
