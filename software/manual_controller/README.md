# manual_controller/

Ubuntu PC に接続したゲームコントローラ(DualSense)で機体(ラズパイ)を
遠隔操作するツール。2台のマシン(機体側・PC側)にまたがる。

```
[Ubuntu PC]                          [ラズパイ (機体)]
 DualSense                            /dev/ttyUSB0 (mob)
    │ pygame                              │ pyserial (MobileBase)
    ▼                                     ▼
remote_client.py  ──── TCP ────▶  remote_server.py
    │                                     │
    └──────── zeroconf(機体を検索) ◀──────┘
```

同一ネットワーク上であることが前提(zeroconf は同一サブネット/マルチキャスト
到達可能な範囲でのみ機能する)。

## ファイル構成

| ファイル | 実行場所 | 役割 |
|---|---|---|
| `remote_protocol.py` | 両方 | 通信メッセージ形式・zeroconfサービス種別の共有定義(依存無し) |
| `remote_controller.py` | 機体側 | ボタン→機体動作の変換ロジック本体(ハード非依存、ユニットテスト済み) |
| `remote_server.py` | 機体側 | zeroconf広告 + TCPサーバー + MobileBase制御(実行スクリプト) |
| `input_mapping.py` | PC側 | pygameの十字キー(hat)/ボタン値 → protocolのボタン名への変換(pygame非依存) |
| `remote_client.py` | PC側 | zeroconf検索 + pygame入力 + TCP送信(実行スクリプト) |
| `dualsense_test.py` | PC側 | コントローラ入力の生確認用(既存、参考にした) |
| `tests/` | どこでも | ユニットテスト(ハード・ネットワーク・pygame不要) |

## セットアップ

機体側(ラズパイ、`software/venv` を使用):

```bash
software/venv/bin/pip install -r software/requirements.txt   # zeroconf を含む
```

PC側(Ubuntu、別環境):

```bash
pip install pygame zeroconf
```

## 実行

機体側:

```bash
software/venv/bin/python3 software/manual_controller/remote_server.py
```

起動時にジャイロキャリブレーション(機体は静止させておくこと)を行ってから
zeroconfサービスを広告し、PCからの接続を待つ。ui_serverが起動していれば
OLEDに状態(待機中/接続中のIP)を表示し、Lボタンで終了できる(`default_app`
のメニューから起動した場合、default_ui は子プロセスの終了を待つだけで
強制終了しないため、これが無いとメニューに戻れなくなる)。ui_serverが無い
環境では自動的にスキップされ、Ctrl+Cで終了する。主なオプション:

```
--mob-port /dev/ttyUSB0      mob シリアルポート
--control-port 50123         PCとのTCPポート
--service-name daylight      zeroconf上のサービス名(PC側と合わせる)
--speed-mmps / --accel-mmps2 / --cell-mm   十字キー上の1区間前進のパラメータ
--no-gyro-calibrate          起動時のGCALを省略
```

PC側:

```bash
python3 software/manual_controller/remote_client.py
```

zeroconfで機体を自動検索して接続する。`--host <IP> --port <PORT>` を指定
すると検索をスキップして直接接続できる。接続が切れても自動的に再検索・
再接続する。

## 操作方法

| ボタン | 動作 |
|---|---|
| 十字キー 上 | 押している間、1区間(既定180mm)前進を繰り返す。離したら現在の区間の前進が完了し次第停止(次の区間には進まない) |
| 十字キー 右 | 押した瞬間に右へ90度その場旋回(1回のみ、離しても何もしない) |
| 十字キー 左 | 押した瞬間に左へ90度その場旋回(同上) |
| 十字キー 下 | 押した瞬間に180度その場旋回(同上) |
| △ | 押している間、低速前進(離すと停止) |
| ○ | 押している間、低速その場右旋回(離すと停止) |
| × | 押している間、低速後退(離すと停止) |
| ▢ | 押している間、低速その場左旋回(離すと停止) |
| L1 / R1 | 未実装(将来: アームサーボ・リロードサーボ・ボールセンサ操作用) |

△○×▢ は mob の LATCH 系コマンド(`LFWD`/`LBACK`/`LTURNL`/`LTURNR`/`LSTOP`、
`software/mob/README.md`)を使う低速連続動作。十字キー上/左右/下は
`MobileBase.stop_at()`/`turn()`(1区間ごとに完全停止・90/180度旋回)を使う。

`stop_at()`/`turn()` は mob の DONE 応答(STOP/TURN は完了後0.5秒の角度
維持ホールドを経てから返る)を待つブロッキング呼び出しのため、前の動作の
完了待ち中は次のコマンドを送れず、1コマンドあたり最大で1秒以上かかる
ことがある(体感レイテンシの主因)。十字キー左/右/下の押下はこの待ち時間
中も FIFO キューに積まれ、取りこぼされずに押した順で実行される
(`remote_controller.py`)。

## 安全設計

- PC側は実際のボタン入力が無くても一定間隔(既定0.2秒)でハートビートを
  送り続ける。機体側はこれが一定時間(既定1.0秒、`remote_server.py` の
  `WATCHDOG_TIMEOUT_S`)途絶えたらリンク切断とみなし、`MobileBase.
  emergency_stop()`(QSTPで減速停止、失敗時は`MOT,0,0`)で緊急停止する。
- TCP切断(PC側のプロセス終了・ネットワーク断)も同様に検知して緊急停止する。
- `MobileBase` の `abort_check` をリンク切断フラグに直結しているため、
  1区間前進や90/180度旋回のDONE待ち中(ブロッキング)でも切断を即座に
  検知して中断できる(micromouseの中断ボタンと同じ仕組み)。

## テスト

```bash
software/venv/bin/python3 -m unittest discover -s software/manual_controller/tests -q
```

`remote_controller.py`(ボタン→動作の変換ロジック、duck-typedなフェイク
base で検証)、`input_mapping.py`(十字キーの hat 値変換)、`remote_protocol.py`
(メッセージのエンコード/デコード)、`remote_server.py` のTCP受信・watchdog・
切断処理(ローカルループバックソケットで検証)をハードウェア・実ネットワーク
・pygame・zeroconf無しでテストできる。
