"""discord_post.py — 俯瞰画像などをDiscordへ投稿する(2026-08-03〜)。

自律開発中、ロボットを動かしたら俯瞰画像をDiscordへ送ってユーザーが
遠隔で状況を把握できるようにする(ユーザー指示、2026-08-03)。Webhook URLの
取得は beacon/discord_ip.py の load_webhook_url() を再利用する。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import requests

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "beacon"))
from discord_ip import load_webhook_url  # noqa: E402


def post_image(image_path: str, content: str = "", *, timeout_s: float = 30.0) -> bool:
    """画像1枚とテキストをDiscordへ投稿する。成功で True。"""
    url = load_webhook_url()
    if not url:
        print("Discord webhook URL 未設定(投稿スキップ)")
        return False
    p = Path(image_path)
    with p.open("rb") as f:
        files = {"file": (p.name, f, "image/jpeg")}
        data = {"content": content[:1900]} if content else {}
        try:
            r = requests.post(url, data=data, files=files, timeout=timeout_s)
        except requests.RequestException as e:
            print(f"Discord投稿失敗: {e}")
            return False
    ok = 200 <= r.status_code < 300
    if not ok:
        print(f"Discord投稿失敗: HTTP {r.status_code} {r.text[:200]}")
    return ok


def post_text(content: str, *, timeout_s: float = 15.0) -> bool:
    url = load_webhook_url()
    if not url:
        return False
    try:
        r = requests.post(url, json={"content": content[:1900]}, timeout=timeout_s)
    except requests.RequestException:
        return False
    return 200 <= r.status_code < 300


if __name__ == "__main__":
    img = sys.argv[1]
    msg = sys.argv[2] if len(sys.argv) > 2 else ""
    print("posted" if post_image(img, msg) else "failed")
