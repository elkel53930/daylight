# beacon

Raspberry Pi 起動時に Discord へ IP アドレスを通知するスクリプト。

## 動作概要

1. Raspberry Pi が起動する
2. ネットワーク（Wi-Fi）接続を待機する
3. インターネット接続を確認する
4. ホスト名・ローカル IP・グローバル IP などを取得する
5. Discord Webhook へ通知を送信する（1回のみ、常駐なし）

systemd の `Type=oneshot` サービスとして動作する。

## 通知例

```
🟢 Raspberry Pi 起動

Host: `rover01`

Time:
2026-07-13 21:15:43 JST

Local IP:
`192.168.1.15`

Global IP:
`126.xxx.xxx.xxx`

Kernel: `6.6.31+rpt-rpi-v8`
OS: Debian GNU/Linux 12 (bookworm)
SSID: `MyNetwork`
```

## ディレクトリ構成

```
beacon/
├── discord_ip.py      # メインスクリプト
├── requirements.txt   # Python 依存パッケージ
├── setup.sh           # インストールスクリプト
├── .env               # Webhook URL（要作成、Git管理外）
└── venv/              # Python 仮想環境（setup.sh が作成）
```

## セットアップ

### 1. リポジトリをクローン

```bash
git clone <repo_url> /home/<user>/beacon
cd /home/<user>/beacon
```

### 2. Webhook URL を設定

以下のいずれかの方法で `DISCORD_WEBHOOK_URL` を設定する。

**A. `.env` ファイル（推奨）**

```bash
echo 'DISCORD_WEBHOOK_URL=https://discord.com/api/webhooks/...' > .env
```

**B. `config.json`**

```json
{
  "DISCORD_WEBHOOK_URL": "https://discord.com/api/webhooks/..."
}
```

**C. 環境変数**

```bash
export DISCORD_WEBHOOK_URL=https://discord.com/api/webhooks/...
```

### 3. インストール

```bash
sudo bash setup.sh
```

実行内容:
- venv の作成
- `requests` のインストール
- systemd サービスファイルの生成・配置
- `systemctl daemon-reload && systemctl enable`

## 動作確認

```bash
# 手動実行
sudo systemctl start discord-startup-notify.service

# ログ確認
journalctl -u discord-startup-notify.service -e
```

正常時のログ出力:

```
Waiting for network...
Network connected.
Sending Discord notification...
Notification sent successfully.
```

## 設定の優先順位

Webhook URL の読み込み順序:

1. 環境変数 `DISCORD_WEBHOOK_URL`
2. `.env` ファイル
3. `config.json`

## 要件

- Raspberry Pi OS (Bookworm)
- Python 3.x
- `iwgetid`（SSID 取得、`wireless-tools` パッケージ）

```bash
sudo apt install wireless-tools
```
