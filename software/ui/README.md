# UI Server

Raspberry Pi CM4 上で動作する UI サーバー。SSD1331 OLED ディスプレイ・タクトスイッチ（L/R）・PWM ブザーを Unix Domain Socket 経由で提供する。

---

## 必要パッケージ

```
lgpio
Pillow
luma.core
luma.oled
msgpack
```

## インストール方法

開発・テスト時は `software/venv` (`default_app/`・`camera/` と共有、詳細は
`software/README.md`) を使う。

```bash
software/venv/bin/pip install -r software/ui/requirements.txt
```

`ui/requirements.txt` は、このディレクトリを単体で `/opt/...` へデプロイ
する場合に何をインストールすべきかの参考リストとしても使える(自動化
されたインストール手順ではなく、手動で環境を作る際の参考)。

## systemd セットアップ

```bash
# サーバーファイルを配置
sudo cp ui_server.py /opt/ui/
sudo cp resources/ /opt/ui/ -r

# サービスファイルを配置して有効化
sudo cp ui_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ui_server
sudo systemctl start ui_server

# ステータス確認
sudo systemctl status ui_server
journalctl -u ui_server -f
```

### ブザーのハードウェアPWM設定

ブザー(GPIO 13)は BCM2711 のハードウェア PWM(PWM0 チャネル1)で駆動する。
`/boot/firmware/config.txt` に以下を追記して再起動しておくこと。

```
dtoverlay=pwm,pin=13,func=4
```

overlay が無い環境では起動時に警告ログを出し、従来どおり lgpio の
ソフトウェア PWM(音程が CPU 負荷の影響を受ける)にフォールバックする。

### 実機での既知の注意点

- `lgpio` は import 時にカレントディレクトリへ通知用パイプファイル
  (`.lgd-nfyN`) を作成する。systemd のデフォルト作業ディレクトリ（`/`）
  では書き込めず失敗するため、`ui_server.service` は
  `WorkingDirectory=/run/ui_server`（`RuntimeDirectory=ui_server` で
  作成される、サービスユーザーが書き込み可能なディレクトリ）を指定
  している。
- 同じ理由で、`UI_SOCKET_PATH` は `/run` 直下ではなく
  `/run/ui_server/ui_server.sock` のように `RuntimeDirectory` 配下を
  指定すること。`/run` 直下は root 以外に書き込み権限がないため、
  非rootユーザーで実行する場合は `bind()` が `PermissionError` になる。
- `User=` を root 以外にする場合は、そのユーザーが `gpio` / `spi`
  グループに属していること（`/dev/gpiochip*` ・SPIデバイスへのアクセス
  に必要）。ハードウェア PWM の sysfs（`/sys/class/pwm/`）は
  Raspberry Pi OS の udev ルール（`99-com.rules`）が export 時に
  `gpio` グループへ書き込み権限を付与するため、サービスユーザーが
  `gpio` グループに属していればよい。ただしこの権限付与は**非同期**で、
  export 直後の書き込みは一時的に `EACCES` になることがある。
  `HardwarePWM` は初期化時にこれを最大2秒リトライする（ここで安易に
  ソフトウェア PWM へフォールバックすると、GPIO 13 を通常出力として
  claim した時点で ALT0 マックスが解除され、以後ハードウェア PWM の
  波形がピンに出なくなるため）。

---

## Socket 仕様

| 項目 | 値 |
|---|---|
| パス | `/tmp/ui_server.sock`（デフォルト）/ `$UI_SOCKET_PATH` で変更可 |
| 種別 | Unix Domain Socket (SOCK_STREAM) |
| フレーム | 4 バイト big-endian 長さ + MessagePack 辞書 |
| 同時接続 | 1 クライアントのみ |

### 優先度制御

- 接続時に `priority` を指定する（0 が最高優先度）
- 現在のクライアントより優先度が高い接続が来た場合、既存クライアントへ `{"status": "PREEMPTED"}` を送信して切断する
- 優先度が同じか低い場合は接続を拒否する

---

## API 一覧

### 接続

```python
from ui_client import UIClient

client = UIClient()
client.connect(priority=3)   # 0 が最高優先度
```

### 切断

```python
client.disconnect()
```

### OLED 表示

```python
from PIL import Image
img = Image.open("image.png").convert("RGB").resize((96, 64))
client.display(img)
```

### OLED クリア

```python
client.clear()
```

### ブザー再生

```python
client.play("ccddeeff")
# c d e f g a b  → 低音 (524–988 Hz)
# C D E F G A B  → 高音 (1048–1976 Hz)
# その他         → 休符
```

### ボタン取得

```python
buttons = client.get_buttons()
# {"left": "released", "right": "pressed"}
# 状態: released / pressed / long_pressed
```

---

## 通信プロトコル詳細

すべてのメッセージは MessagePack 辞書形式。各メッセージの前に 4 バイト big-endian の長さプレフィックスが付く。

### connect

```
→ {"cmd": "connect", "priority": 3}
```

### display

```
→ {"cmd": "display", "width": 96, "height": 64, "image": <bytes>}
← {"status": "ok"}
```

`image` は RGB888 生バイト列 (96 × 64 × 3 = 18432 バイト)。

### clear

```
→ {"cmd": "clear"}
← {"status": "ok"}
```

### buttons

```
→ {"cmd": "buttons"}
← {"left": "released", "right": "pressed"}
```

### play

```
→ {"cmd": "play", "melody": "ccddeeff"}
← {"status": "ok"}
```

### PREEMPTED（サーバーからの通知）

```
← {"status": "PREEMPTED"}
```

---

## サンプル実行方法

```bash
# PNG 画像を表示
python3 examples/display_image.py image.png

# ボタン状態を 100ms 毎に表示
python3 examples/button_test.py

# サンプルメロディを再生
python3 examples/melody_test.py
```

## テスト実行

```bash
python3 -m pytest tests/test_protocol.py -v
# または
python3 tests/test_protocol.py
```
