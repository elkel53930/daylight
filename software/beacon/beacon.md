# Discord IP通知機能 実装仕様書

## 目的

Raspberry Pi Compute Module 4 の起動後、Wi-Fi経由でインターネット接続が利用可能になった時点で、Discordへ起動通知およびIPアドレス情報を送信する。

---

# 動作要件

起動時に以下の処理を実施する。

1. Raspberry Piが起動する。
2. ネットワーク接続（Wi-Fi）が確立するまで待機する。
3. インターネットへ接続可能であることを確認する。
4. IPアドレス情報を取得する。
5. Discord Webhookへ通知を送信する。
6. 処理終了後は常駐しない。

systemdの`Type=oneshot`サービスとして動作すること。

---

# 開発環境

* OS: Raspberry Pi OS (Bookworm)
* Python: 3.x
* Pythonはプロジェクト内のvenvを使用すること
* パッケージ管理: pip

---

# ディレクトリ構成

以下を前提とする。

```text
<project_root>/
├── venv/
├── discord_ip.py
└── requirements.txt
```

systemdからは

```text
<project_root>/venv/bin/python
```

を使用して実行すること。

---

# Pythonライブラリ

最低限以下を使用する。

* requests

必要に応じて標準ライブラリを使用してよい。

---

# 通知タイミング

通知は

* 起動後
* network-online.target到達後

に一度だけ行う。

通知失敗時はエラーログを出力して終了する。

再試行は不要。

---

# インターネット接続確認

インターネット接続確認はTCP接続またはHTTPアクセスにより行う。

例

* 8.8.8.8:53
* https://api.ipify.org

ネットワーク未接続の場合は数秒間隔でリトライする。

---

# 取得する情報

以下を取得すること。

| 項目                  | 必須 |
| ------------------- | -- |
| Hostname            | ○  |
| Local IPv4 Address  | ○  |
| Global IPv4 Address | ○  |
| 起動日時(JST)           | ○  |

可能なら以下も取得する。

* Linuxカーネルバージョン
* Raspberry Pi OS情報
* 接続SSID

---

# Discord通知フォーマット

以下のような見やすい形式とする。

```text
🟢 Raspberry Pi 起動

Host: rover01

Time:
2026-07-13 21:15:43 JST

Local IP:
192.168.1.15

Global IP:
126.xxx.xxx.xxx
```

Markdown記法を利用して読みやすくしてよい。

---

# 設定ファイル

Webhook URLはソースコードへ直接書き込まないこと。

以下のいずれかで管理すること。

優先順位

1. 環境変数
2. .envファイル
3. config.json

コード内へのハードコーディングは禁止。

---

# ログ

標準出力・標準エラーへ出力し、journalctlから確認できること。

ログ例

```text
Waiting for network...
Network connected.
Sending Discord notification...
Notification sent successfully.
```

---

# systemdサービス

以下のようなサービスを作成すること。

```ini
[Unit]
Description=Discord Startup Notification
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=<username>
WorkingDirectory=<project_root>
ExecStart=<project_root>/venv/bin/python <project_root>/discord_ip.py

[Install]
WantedBy=multi-user.target
```

実際のパスはインストール時に自動生成または設定可能とすること。

---

# インストールスクリプト

以下を自動で実施するセットアップスクリプトを作成すること。

* venv確認
* 必要パッケージインストール
* systemdサービス生成
* systemctl daemon-reload
* systemctl enable
* 動作確認方法の表示

---

# requirements.txt

最低限

```text
requests
python-dotenv
```

を含めること。

---

# エラー処理

以下のケースを適切に処理すること。

* Webhook URL未設定
* インターネット未接続
* Global IP取得失敗
* Discord APIエラー
* HTTPタイムアウト

異常終了時は終了コード0以外を返すこと。

---

# コーディング方針

* Python 3の標準的な書き方とする。
* 関数へ適切に分割する。
* 型ヒントを付与する。
* docstringを付与する。
* マジックナンバーを避ける。
* 定数を冒頭へまとめる。
* 例外は適切に処理する。

---

# 成果物

AIエージェントは以下を作成すること。

* discord_ip.py
* requirements.txt
* .env.example
* install.sh
* uninstall.sh
* discord-ip.service（テンプレートまたは生成処理）
* README.md

README.mdには以下を記載すること。

* セットアップ方法
* Webhook設定方法
* venv作成方法
* サービス有効化方法
* ログ確認方法
* アンインストール方法

---

# 完了条件

以下がすべて満たされること。

* venv上で正常動作する。
* systemdから自動起動する。
* 起動時にDiscordへ通知が届く。
* ログがjournalctlで確認できる。
* Webhook URLをコード変更なしで設定できる。
* READMEの手順だけで第三者がセットアップ可能である。
