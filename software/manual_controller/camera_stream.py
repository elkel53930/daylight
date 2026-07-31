"""機体カメラの映像をHTTPで配信する(手動操作中の状況確認用)。

remote_server.py から呼ばれる。software/vision/vision_http_test.py と同じ
仕組み(Picamera2 + 標準ライブラリ http.server、検出オーバーレイ無しの生映像)
だが、検出処理を行わない分さらに高フレームレートになる。ブラウザで
`http://<機体IP>:<カメラポート>/` を開くと、自動更新される<img>タグで
継続的に映像が表示される(MJPEGではなく短間隔でのJPEGポーリング。
vision_http_test.pyで実測5.5fps@640x480、検出処理が無い分さらに速い)。

カメラの向きはFutabaアームサーボと連動している(software/arm/
futaba_servo.py)。走行中はアームサーボがホーム位置(論理角度0度=前方固定)
のため前方の映像になるが、L1のボール回収シーケンス中はアームが動くため
映像の向きも一時的に変わる。
"""

from __future__ import annotations

import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
JPEG_QUALITY = 80
POLL_INTERVAL_MS = 150  # ブラウザ側の再取得間隔

PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<title>Daylight camera</title></head><body style="margin:0;background:#111">
<img id="v" style="width:100%;max-width:960px;display:block;margin:auto">
<script>
function tick(){document.getElementById('v').src='/frame.jpg?'+Date.now();}
setInterval(tick,""" + str(POLL_INTERVAL_MS).encode() + b""");tick();
</script></body></html>"""


class CameraSource:
    """Picamera2 を1つ起動して使い回す(スレッド安全のためロックで保護)。"""

    def __init__(self, width: int = CAPTURE_WIDTH, height: int = CAPTURE_HEIGHT):
        from picamera2 import Picamera2

        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._cam.configure(cfg)
        self._cam.start()
        time.sleep(0.5)  # 露出安定待ち
        self._lock = threading.Lock()

    def capture_jpeg(self, quality: int = JPEG_QUALITY) -> bytes:
        from PIL import Image
        import numpy as np

        with self._lock:
            bgr = self._cam.capture_array("main")
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        img = Image.fromarray(rgb).convert("RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

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
                    jpeg = source.capture_jpeg()
                except Exception as e:  # 撮影失敗でもサーバは落とさない
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


def start_camera_server(
    host: str, port: int
) -> Tuple[Optional[ThreadingHTTPServer], Optional[CameraSource]]:
    """カメラ映像配信サーバーを起動する。

    カメラに接続できない場合は警告を出して (None, None) を返す(他の操作には
    影響しない)。成功時は呼び出し側で stop_camera_server() を呼んで
    後始末すること。
    """
    try:
        source = CameraSource()
    except Exception as e:
        print(f"# カメラに接続できません(映像配信は無効): {e}")
        return None, None

    try:
        server = ThreadingHTTPServer((host, port), _make_handler(source))
    except OSError as e:
        print(f"# カメラ配信サーバーの起動に失敗: {e}")
        source.close()
        return None, None

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, source


def stop_camera_server(
    server: Optional[ThreadingHTTPServer], source: Optional[CameraSource]
) -> None:
    if server is not None:
        server.shutdown()
        server.server_close()
    if source is not None:
        source.close()
