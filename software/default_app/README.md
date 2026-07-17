# Robot Default UI

ロボットの「ホーム画面」に相当する常駐アプリケーション。`ui_server` に
`priority=100`（最低優先度）で接続し、通常時のシステム状態表示・アプリ
ケーションランチャー・システム操作（再起動/シャットダウン）を提供する。

他のアプリケーションが `ui_server` へ接続すると Default UI は UI 所有権
を失う（`PREEMPTED`）。これは正常な動作であり、Default UI はクラッシュ
せず、UI が空いたら自動的に再接続してメイン画面へ戻る。

---

## 必要環境

- Raspberry Pi 4
- Raspberry Pi OS Bookworm
- 稼働中の `ui_server`（`../ui/ui_server.py`）
- Python 3

## 必要パッケージ

```
PyYAML
Pillow
msgpack
```

`ui_client.py` は本アプリケーションと同一マシン上の `ui_server` プロジェ
クトから利用する。開発時はリポジトリのレイアウト（`software/default_app`
と `software/ui` が兄弟ディレクトリ）に従うことで自動的に解決される。
デプロイ時は `ui_client.py` を `PYTHONPATH` に含めるか、本ディレクトリへ
コピーすること。

同様に `discord_alerts.py` は `software/beacon/discord_ip.py`
（`load_webhook_url`）に依存する。開発時はリポジトリのレイアウト
（`software/default_app` と `software/beacon` が兄弟ディレクトリ）で自動
解決されるが、デプロイ時は `beacon/discord_ip.py` を `PYTHONPATH` に含め
るか、本ディレクトリへコピーすること。

---

## インストール

```bash
cd software/default_app
pip install -r requirements.txt
```

## 設定

アプリケーション一覧は YAML で管理する。

```bash
sudo mkdir -p /etc/robot-ui
sudo cp config/applications.yaml.example /etc/robot-ui/applications.yaml
```

### アプリケーション追加

`applications.yaml` を編集するだけで追加できる。Default UI のコード変更
は不要。

```yaml
applications:
  - name: Example
    command:
      - python3
      - /opt/robot/apps/example.py
    priority: 10
```

- `name`: メニューに表示される名前（必須）
- `command`: 引数リスト形式のコマンド（`shell=False` で実行、必須）
- `priority`: アプリケーションが `ui_server` に接続する際の優先度（省略時 20）

構文エラーや不正なエントリがあっても Default UI 全体は停止せず、該当
エントリのみスキップされる（journal に警告が出力される）。

### バッテリー電圧監視

MCP3221 の I2C アドレスは実機配線に合わせて `battery.py` の
`DEFAULT_I2C_ADDRESS` を変更するか、`BatteryMonitor` へ独自の
`MCP3221Reader` を渡すこと。

### Discord 通知（バッテリー低下 / CPU 温度上昇）

`discord_alerts.py` がバッテリー電圧・CPU 温度を独自のバックグラウンド
スレッドで監視し、以下のいずれかを満たすと Discord Webhook へ通知する。

- バッテリー電圧が 6.5V 未満（`BatteryMonitor.is_low` と同じ閾値。6.7V
  まで回復するとクリアされ、次に低下したら再通知する）
- CPU 温度が 75℃ を超過（72℃ まで下がるとクリアされ、次に超過したら
  再通知する）

`ui_server` への接続状態や Default UI のメインループとは独立したスレッ
ドで動作するため、アプリ起動待ちで `ui_server` から切断している間も
監視・通知を継続する。

Webhook URL の設定方法は `camera_discord.py` / `beacon/discord_ip.py` と
共通（優先順）。

1. 環境変数 `DISCORD_WEBHOOK_URL`
2. `beacon/.env`
3. `beacon/config.json`

いずれも未設定の場合は起動時に警告ログを出し、通知機能のみ無効化される
（Default UI 自体は継続動作する）。

### ブザーのメロディ

ボタン操作音・アプリ起動音・低電圧警告音は、すべて `melodies.py` に
まとめてある。音を変更したい場合はこのファイルだけを編集すればよい。

---

## 権限設定（sudoers）

Default UI 自体は root で常駐させる必要はないが、以下の操作は権限が要る。

- CPU 周波数取得: `cat /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq`
  （カーネルによってはこのファイルは root しか読めないため）
- 再起動: `systemctl reboot`
- シャットダウン: `systemctl poweroff`

いずれもアプリケーションは `sudo` 経由でコマンドを実行するので、サービス
を動かすユーザーに対してパスワード無しで実行できるよう sudoers に登録
しておく。

```bash
sudo visudo -f /etc/sudoers.d/robot-default-ui
```

```text
youruser ALL=(root) NOPASSWD: /usr/bin/cat /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq
youruser ALL=(root) NOPASSWD: /usr/bin/systemctl reboot
youruser ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff
```

`youruser` は `default-ui.service` を実行するユーザーに置き換えること。
これらのコマンドが失敗しても Default UI は継続動作し、CPU 周波数は
`N/A`、再起動/シャットダウンは失敗ログを出してメニューへ戻る。

---

## systemd 登録

```bash
sudo cp default-ui.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable default-ui.service
sudo systemctl start default-ui.service
```

`default-ui.service` の `UI_SOCKET_PATH` は `ui_server.service` 側と
必ず一致させること（デフォルトは `/run/ui_server/ui_server.sock`）。
`/run` 直下は非rootユーザーから書き込めないため、`ui_server` 側の
`RuntimeDirectory=ui_server` 配下のパスを指定する必要がある。

`ui_server.service` の起動後に自動起動し、異常終了時は `Restart=always`
により再起動される。

## ログ確認

```bash
journalctl -u default-ui.service -f
```

---

## 操作方法

2 ボタンのみで操作する。

| ボタン | 機能 |
|---|---|
| L | 次のメニュー項目へ（循環） |
| R | 現在の項目を決定 |

メイン画面:

```text
IP: 192.168.1.10
BAT: 7.42V
> Applications
  System
```

IP アドレスは常時表示、2 段目は Battery / CPU 温度 / CPU 周波数を
2 秒間隔で切り替えて表示する。

`Applications` / `System` メニューには一覧の最後に `Back` があり、選択
するとメイン画面へ戻る。`Reboot` / `Shutdown` は確認画面（L: No / R: Yes）
を経由してから実行される。

---

## テスト

```bash
python3 -m pytest tests/ -v
```

---

## トラブルシューティング

**`ui_server` に接続できない**
`ui_server.service` が起動しているか確認する（`systemctl status
ui_server`）。Default UI は 1 秒間隔でリトライを続けるため、`ui_server`
が復帰すれば自動的に再接続する。

**MCP3221 が読めない**
`/dev/i2c-1` の存在と I2C の有効化（`raspi-config`）、配線、I2C アドレス
を確認する。読み取りに失敗してもバッテリー表示が `N/A` になるだけで
Default UI 自体は継続動作する。

**IP アドレスが表示されない**
`wlan0` が存在し、IP を取得済みか確認する（`ip addr show wlan0`）。未取得
時は `IP: N/A` と表示される。

**CPU 温度が取得できない**
`vcgencmd` が利用可能か確認する。失敗時は `CPU: N/A` と表示される。

**Discord 通知が届かない**
`DISCORD_WEBHOOK_URL`（環境変数 / `beacon/.env` / `beacon/config.json`）
が設定されているか確認する。未設定時は起動時に警告ログのみで通知は
送信されない。設定済みでも届かない場合は Webhook URL 自体の有効性を
`camera_discord.py` 等で確認すること。

**アプリケーションが起動しない**
`/etc/robot-ui/applications.yaml` の `command` が実行可能なパスを指して
いるか確認する。journal にエラーが出力される。

**PREEMPTED 後に再接続できない**
他のアプリケーションが `ui_server` を保持し続けていないか確認する
（アプリ終了時に UI を解放しているか）。`ui_server` 自体が再起動した
場合も Default UI は自動的に再接続する。
