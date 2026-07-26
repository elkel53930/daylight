#!/usr/bin/env python3
"""CSIカメラの性能検証用: 最大解像度・最大画角で1秒ごとにHTTP配信する。

software/manual_controller/camera_stream.py と同じ「ブラウザで自動更新
される<img>タグ」方式だが、こちらは性能検証が目的:

  - 解像度はセンサーのフル解像度(実機で `Picamera2().sensor_modes` を
    確認: Camera Module 3 Wide/imx708_wide は 4608x2592 が最大)。
  - 画角も最大になるモードを使う。sensor_modes の crop_limits を見ると
    1536x864 モードだけ crop_limits が (768,432,3072,1728) に狭まり
    画角が狭い(2倍ビニング+クロップ)のに対し、2304x1296 と 4608x2592 は
    crop_limits が (0,0,4608,2592)(フルセンサー領域)で画角が最大になる。
    そのため単に解像度を上げるだけでなく、4608x2592 を明示的に指定して
    フル画角のモードを選ばせる必要がある。
  - 配信間隔は1秒(高解像度JPEGエンコード・転送の負荷を確認する目的、
    camera_stream.py の 150ms 更新とは別用途)。
  - 撮影・エンコードにかかった時間をコンソールとHTTPページ上に表示する。

使い方:
    software/venv/bin/python3 software/camera/camera_max_res_test.py

起動後に表示される http://<機体IP>:8081/ をブラウザで開く。
"""

from __future__ import annotations

import argparse
import io
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Tuple

CAPTURE_WIDTH = 4608
CAPTURE_HEIGHT = 2592
JPEG_QUALITY = 90
POLL_INTERVAL_MS = 1000

PAGE_TEMPLATE = """<!doctype html><html><head><meta charset="utf-8">
<title>Daylight camera (max res test)</title></head>
<body style="margin:0;background:#111;color:#eee;font-family:monospace">
<div id="info" style="padding:4px 8px">-</div>
<img id="v" style="width:100%%;display:block;margin:auto">
<script>
function tick(){
  var img = document.getElementById('v');
  var t0 = performance.now();
  var url = '/frame.jpg?' + Date.now();
  img.onload = function(){
    document.getElementById('info').textContent =
      'client fetch: ' + Math.round(performance.now() - t0) + 'ms  ' + url;
  };
  img.src = url;
}
setInterval(tick, %d);
tick();
</script></body></html>""" % POLL_INTERVAL_MS


def get_local_ip() -> str:
    """LAN上でこの機体が実際に使っているIPを取得する(UDP接続トリック、パケットは送らない)。"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]


class CameraSource:
    def __init__(self, width: int, height: int, quality: int):
        from picamera2 import Picamera2

        self._quality = quality
        self._cam = Picamera2()
        cfg = self._cam.create_still_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._cam.configure(cfg)
        self._cam.start()
        time.sleep(1.0)  # 露出安定待ち
        actual = self._cam.camera_configuration()["main"]["size"]
        print(f"# camera configured: main={actual}")

    def capture_jpeg(self) -> Tuple[bytes, float, float]:
        from PIL import Image
        import numpy as np

        t0 = time.monotonic()
        bgr = self._cam.capture_array("main")
        t1 = time.monotonic()
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        img = Image.fromarray(rgb).convert("RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=self._quality)
        t2 = time.monotonic()
        return buf.getvalue(), t1 - t0, t2 - t1

    def close(self) -> None:
        try:
            self._cam.stop()
        except Exception:
            pass


def _make_handler(source: CameraSource):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # アクセスログを抑制
            pass

        def do_GET(self):
            if self.path.startswith("/frame.jpg"):
                try:
                    jpeg, capture_s, encode_s = source.capture_jpeg()
                except Exception as e:  # 撮影失敗でもサーバーは落とさない
                    self.send_error(500, str(e))
                    return
                total_ms = (capture_s + encode_s) * 1000
                print(
                    f"# frame: {len(jpeg) / 1024:.0f}KB  "
                    f"capture={capture_s * 1000:.0f}ms  encode={encode_s * 1000:.0f}ms  "
                    f"total={total_ms:.0f}ms"
                )
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Capture-Ms", f"{capture_s * 1000:.0f}")
                    self.send_header("X-Encode-Ms", f"{encode_s * 1000:.0f}")
                    self.end_headers()
                    self.wfile.write(jpeg)
                except (BrokenPipeError, ConnectionResetError):
                    # クライアントが既に接続を切っている(上記コメント参照)。
                    pass
            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGE_TEMPLATE.encode("utf-8"))

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--width", type=int, default=CAPTURE_WIDTH)
    parser.add_argument("--height", type=int, default=CAPTURE_HEIGHT)
    parser.add_argument("--quality", type=int, default=JPEG_QUALITY, help="JPEGエンコード品質(1-95)")
    args = parser.parse_args()

    source = CameraSource(args.width, args.height, args.quality)
    try:
        server = ThreadingHTTPServer((args.host, args.port), _make_handler(source))
    except OSError as e:
        print(f"# サーバー起動に失敗: {e}")
        source.close()
        return

    print(f"# http://{get_local_ip()}:{args.port}/ を開いてください(Ctrl+Cで終了)")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        source.close()


if __name__ == "__main__":
    main()
