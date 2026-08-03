"""overhead.py — 俯瞰カメラ(x13u に接続された Logitech C270)から静止画を取得する。

C270 はコース全体とロボットを俯瞰する位置に設置されており、走行結果の
真値(絶対位置・向き)の確認に使う。x13u 上で ffmpeg で1フレーム撮影し、
scp でローカルへ取ってくる。

注意: x13u にはノートPC内蔵カメラ(Integrated RGB Camera)が video0〜3 に
存在するため、C270 は必ず by-id か /dev/video4 で指定する。
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

X13U_HOST = "k-iida@x13u.local"
C270_DEVICE = "/dev/video4"  # by-id: usb-046d_C270_HD_WEBCAM_*-video-index0
REMOTE_TMP = "/tmp/c270_fastrun.jpg"


def capture(local_path: str, *, video_size: str = "1280x960", timeout_s: float = 20.0) -> str:
    """C270 で1フレーム撮影して local_path へ保存し、そのパスを返す。

    C270 の実解像度は 1280x960(Quad-VGA)。HD(1280x720)で撮ると迷路の一部が
    欠けるので必ず 1280x960 で撮る(2026-08-03 ユーザー指摘)。1マス=180mm、
    4x4 マス、カメラはほぼ真上設置で中心4マスはほぼ真上見下ろし。Discord へ
    投稿するときは VGA(640x480)に縮小すること。
    """
    remote_cmd = (
        f"ffmpeg -y -f v4l2 -input_format mjpeg -video_size {video_size} "
        f"-i {C270_DEVICE} -frames:v 1 {REMOTE_TMP} 2>/dev/null && echo OK"
    )
    r = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", X13U_HOST, remote_cmd],
        capture_output=True, text=True, timeout=timeout_s,
    )
    if "OK" not in r.stdout:
        raise RuntimeError(f"俯瞰撮影に失敗: {r.stderr.strip()[:200]}")
    Path(local_path).parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["scp", "-q", f"{X13U_HOST}:{REMOTE_TMP}", local_path],
        check=True, timeout=timeout_s,
    )
    return local_path


def capture_and_post(content: str, *, work_dir: str = "/tmp") -> None:
    """俯瞰を 1280x960 で撮影し、VGA(640x480)へ縮小して Discord へ投稿する。

    ロボットを動かしたら必ずこれで投稿する(2026-08-03 ユーザー指示: 動かしたら
    俯瞰を VGA に縮小して Discord へ)。
    """
    import sys as _sys
    from PIL import Image

    ts = int(time.time())
    full = f"{work_dir}/overhead_{ts}.jpg"
    vga = f"{work_dir}/overhead_{ts}_vga.jpg"
    capture(full)
    img = Image.open(full)
    img.thumbnail((640, 480))
    img.save(vga, quality=85)

    _sys.path.insert(0, str(Path(__file__).resolve().parent))
    from discord_post import post_image
    post_image(vga, content)


if __name__ == "__main__":
    import sys

    out = sys.argv[1] if len(sys.argv) > 1 else f"/tmp/c270_{int(time.time())}.jpg"
    print(capture(out))
