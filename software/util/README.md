# util

mob (ESP32-S3) とシリアル通信して、個別のハードウェア機能を単体で
確認・操作するための診断用CLIスクリプト置き場。

いずれも `software/venv` の Python で実行する(pyserial が必要)。

```bash
software/venv/bin/python3 software/util/<script>.py [オプション]
```

## ファイル構成

| ファイル | 説明 |
|----------|------|
| `servo_control.py` | RCサーボ(IO1)を角度指定またはトルクオフで操作する |
| `ball_check.py` | ボールセンサ(IO14)のADC生値を一定間隔で表示する |

### servo_control.py

```bash
software/venv/bin/python3 software/util/servo_control.py --angle 90
software/venv/bin/python3 software/util/servo_control.py --off
```

mob の `SRV,<angle>` / `SRVOFF` コマンドを送信する。`--angle` と `--off` は排他。

### ball_check.py

```bash
software/venv/bin/python3 software/util/ball_check.py
software/venv/bin/python3 software/util/ball_check.py --interval 0.2
```

mob の `SEN` コマンドを `--interval` 秒毎(既定0.5秒)に発行し、
レスポンス末尾の `ball_raw`(ADC生値)と `ball_det`(検出フラグ)を表示する。

## 共通オプション

各スクリプトとも `--port`(既定 `/dev/ttyUSB0`)と `--baud`(既定 `3000000`)で
mob のシリアルポート・ボーレートを指定できる。
