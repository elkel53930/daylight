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
| `param_tui.py` | チューニングパラメータをTUIで調整・NVS保存する(SEN値も表示) |

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

### param_tui.py

```bash
software/venv/bin/python3 software/util/param_tui.py
```

mob の `PGET`/`PSET`/`PSAVE`/`PLOAD`/`PRESET`(software/mob/params.h)を
curses の TUI で操作する。パラメータ一覧はハードコードせず起動時に
`PGET` で mob から取得するため、ファームウェア側で `params.h` にパラメータ
を追加/削除しても本ツールの修正は不要。上部に `SEN` の値(バッテリー電圧・
壁センサ・エンコーダ・オドメトリ・ボールセンサ)を約5Hzで表示し続ける。

| キー | 動作 |
|---|---|
| `↑`/`↓` | パラメータ選択 |
| `←`/`→` | 選択中の値を ±ステップ で変更(即 `PSET`、RAMのみ反映) |
| `Enter` | 選択中の値を直接入力して変更(即 `PSET`) |
| `[` / `]` | ステップを ÷10 / ×10 |
| `s` | `PSAVE`(現在値を機体のNVSへ恒久保存) |
| `l` | `PLOAD`(NVSから再読込) |
| `r` | `PRESET`(RAM値をビルド時デフォルトへ、NVSは無変更) |
| `g` | `PGET` で一覧を再取得(表示をmobの実際値に同期) |
| `/` | 名前でフィルタ(Enterで確定、Escで解除) |
| `q` | 終了 |

## 共通オプション

各スクリプトとも `--port`(既定 `/dev/ttyUSB0`)と `--baud`(既定 `3000000`)で
mob のシリアルポート・ボーレートを指定できる。
