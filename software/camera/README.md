# camera/

CSIカメラの動作確認・Discord投稿用の手動実行スクリプト。systemdサービス
化はされていない。

- `camera_test.py` — カメラ映像をリアルタイムでOLED(`ui_server`経由)に表示する
- `camera_discord.py` — 1フレーム撮影してDiscordに投稿する(webhook設定は
  `beacon/discord_ip.py` と共通、`default_app/README.md` 参照)

## インストール・実行

`software/venv` (`default_app/`・`ui/` と共有、詳細は
`software/README.md`) を使う。`picamera2` はpipではなくapt経由で入れる。

```bash
sudo apt install -y python3-picamera2
software/venv/bin/pip install -r software/requirements.txt

software/venv/bin/python3 software/camera/camera_test.py --once
software/venv/bin/python3 software/camera/camera_discord.py --message "test"
```

`camera_test.py` の実行には `ui_server` が起動している必要がある。
