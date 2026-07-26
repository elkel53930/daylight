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
| `ball_pickup.py` | 機体側 | L1のボール回収シーケンス本体(ハード非依存、duck-typedなbase/arm) |
| `remote_server.py` | 機体側 | zeroconf広告 + TCPサーバー + MobileBase/Futabaアーム制御(実行スクリプト) |
| `input_mapping.py` | PC側 | pygameの十字キー(hat)/ボタン値 → protocolのボタン名への変換、受信行分割・振動(rumble)処理(pygame非依存) |
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
| L1 | 押した瞬間にボール回収シーケンスを1回実行(下記、数秒かかる) |
| R1 | 押した瞬間にリロードサーボを180度へ(1回のみ、離しても何もしない) |

△○×▢ は mob の LATCH 系コマンド(`LFWD`/`LBACK`/`LTURNL`/`LTURNR`/`LSTOP`、
`software/mob/README.md`)を使う低速連続動作。十字キー上/左右/下は
`MobileBase.stop_at()`/`turn()`(1区間ごとに完全停止・90/180度旋回)を使う。

### L1: ボール回収シーケンス(`ball_pickup.py`)

1. アームサーボ(Futaba)・リロードサーボ(mob SRV)を0度へ
2. 0.5秒待つ
3. リロードサーボを140度へ
4. アームサーボを1000msかけて103度へ
5. アームサーボが103度に到達したらファンDuty 50%
6. ボールセンサ値(ball_raw)が100を3回連続で超えたらアームサーボを
   1000msかけて0度へ。100を超えずに2秒経過したらリロードサーボを0度に
   戻して失敗終了(以降の手順はスキップ)
7. アームサーボが0度に到達したらファンDuty 0%
8. 最終確認でball_rawが100未満ならリロードサーボを0度へ戻す(失敗)
9. 100以上ならボール保持成功(`RemoteController.ball_held = True`、
   OLEDに `BALL: OK` 表示)

アームサーボが未接続(Futaba接続失敗)の場合、L1は警告を出して何もしない
(他の操作には影響しない)。`software/arm/ball_sequence.py`(単体スクリプト
版)とは似ているが別物で、こちらは連続検出の判定・タイムアウト時間が異なる
(リモート操作用に個別に設計)。

`stop_at()`/`turn()` は mob の DONE 応答(STOP/TURN は完了後0.5秒の角度
維持ホールドを経てから返る)を待つブロッキング呼び出しのため、前の動作の
完了待ち中は次のコマンドを送れず、1コマンドあたり最大で1秒以上かかる
ことがある(体感レイテンシの主因)。十字キー左/右/下の押下はこの待ち時間
中も FIFO キューに積まれ、取りこぼされずに押した順で実行される
(`remote_controller.py`)。

### 完了時の振動フィードバック

十字キー上(1区間前進)・左/右/下(90/180度旋回)が**成功**完了するたびに、
機体からPCへ振動通知(`{"type": "rumble", "duration_ms": 100}`)を送り、
コントローラを0.1秒振動させる。JOG系(△○×▢、押しっぱなし)・L1・R1では
送らない(完了タイミングが曖昧、または元々押しっぱなしで完了概念が無い
ため)。失敗(リンク切断等で`AbortRequested`/`MobileBaseError`)した場合も
送らない。

- 機体側: `RemoteController(..., on_command_done=...)` に渡したコールバックが
  成功時のみ呼ばれる(`remote_controller.py`)。`remote_server.py` は
  `queue.Queue` 経由でこれを `handle_client()` に渡し、TCP で送信する。
- PC側: `remote_client.py` が `select` で受信データを非ブロッキングに確認し、
  rumble通知を受けたら `joystick.rumble(1.0, 1.0, duration_ms)` を呼ぶ。
  pygameの`rumble()`の`duration_ms`引数は無視される既知の不具合があるため、
  `duration_ms`後に明示的に`stop_rumble()`を呼んで止める。

### 制御収束待ち(STOP_HOLD)の一時短縮

STOP/TURN完了後、mob側は角度整定のため`params.stop_hold_sec`(既定0.5秒)
だけホールドしてからDONEを返す(CLAUDE.md参照、慣性回転中の角度誤取り込み
防止のための意図的な仕様)。手動操作ではこれが体感レイテンシの主因になる
ため、`remote_server.py`は起動時にPGET/PSETで`stop_hold_sec`を一時的に
0.05秒へ短縮し(NVSには保存しないためRAM上のみ、mob再起動で既定値に戻る)、
終了時に元の値へ復元する。復元はSIGTERM/SIGINTどちらでも実行される
(SIGTERMは内部でKeyboardInterruptに変換して同じ終了経路に合流させている)。

⚠️ SIGKILL・電源断など復元コードが一切走らない終了のさせ方をすると、mobの
電源が入ったままの間(次にmicromouseの探索走行等を行うときも)ホールド
時間が短いままになり、旋回精度に影響しうる。気づいた場合はmobを再起動する
か、`software/util/param_tui.py`等で`stop_hold_sec`を0.5へ手動で戻すこと。

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
base で検証。on_command_done コールバックが成功時のみ呼ばれることも含む)、
`ball_pickup.py`(L1シーケンス、sleep/nowを差し替え可能な偽時計で実時間
待機なしに手順・タイミング・分岐を検証)、`input_mapping.py`(十字キーの
hat 値変換、受信行分割、rumble通知の解釈)、`remote_protocol.py`
(メッセージのエンコード/デコード)、`remote_server.py` のTCP受信・
watchdog・切断処理・振動通知の送信(ローカルループバックソケットで検証)を
ハードウェア・実ネットワーク・pygame・zeroconf無しでテストできる。
