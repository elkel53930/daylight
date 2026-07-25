# CLAUDE.md — Daylight 開発知見

このリポジトリで作業する際の前提知識。2026-07-17〜18 の開発セッション
(マイクロマウス実装、ブザーのハードウェア PWM 化)で得た知見をまとめたもの。
マイクロマウスの詳細な経緯は `micromouse_implementation_report.md` を参照。

## リポジトリ概要

RoboSweep エージェント **Daylight**(Twilight の後継機)のリポジトリ。
開発・実行環境はロボット搭載の Raspberry Pi CM4 上(このリポジトリを clone
した場所がそのまま実機)。

```
elec/       KiCad 基板データ
software/
  default_app/  ホーム画面アプリ(systemd: default-ui.service、稼働中)
  ui/           OLED/ボタン/ブザーの UI サーバー(systemd: ui_server.service、稼働中)
  micromouse/   マイクロマウス自律走行アプリ(2026-07 実装)
  camera/       CSI カメラスクリプト(手動実行)
  beacon/       起動時 Discord IP 通知
  mob/          ESP32-S3 ファームウェア(Arduino/C++、モータ・センサ制御)
```

参考実装(前身機): https://github.com/elkel53930/robosweep_twilight
— mob のシリアルプロトコルはほぼ共通。迷路アルゴリズム・走行ループの
設計思想の出典でもある。

## ハードウェア構成の要点

- **mob (ESP32-S3)** が全ての低レベル制御を持つ: 1kHz 制御ループ、車輪速度
  PID、ジャイロ角度/角速度 FB、壁センサ横補正、台形加速の距離走行。
  ラズパイはシリアルでコマンドを送るだけ。
- 接続: `/dev/ttyUSB0`(FTDI FT231X)、**3,000,000 bps**。
- 壁センサは **3個**(前・左横・右横、LED ON/OFF 差分方式)。
  SEN 応答の `lf`/`rf` は**どちらも前センサ値**(Twilight 4センサとの互換のため)。
- エンコーダ AS5047(14bit 生値 0–16383)、IMU LSM6DSR(gyro z のみ使用)。
- UI: OLED 96×64 + ボタン L/R + ブザー。`ui_server` に Unix ドメインソケットで
  接続(4バイト長プレフィックス + MessagePack)。優先度制御あり
  (default_app は priority=100 で最低、アプリは 20 前後で奪取)。
- ブザー(GPIO 13)は BCM2711 ハードウェア PWM(PWM0 ch1、sysfs 経由)駆動
  (2026-07-18〜)。config.txt に `dtoverlay=pwm,pin=13,func=4` 設定済み。
  ⚠️ **lgpio 等で GPIO 13 を出力として claim しないこと** — ALT0 マックスが
  解除され、PWM 波形がピンに出なくなる(無音。復旧は `pinctrl set 13 a0`)。
  sysfs 権限は udev が export 後に非同期付与するため直後の書き込みは
  EACCES になり得る。詳細は `software/ui/README.md`。

## mob シリアルプロトコルの落とし穴

- 現行 `mob.ino` の SEN 応答は **12 フィールド**(2026-07-24〜、ボールセンサ拡張):
  `SEN,gyro[rad/s],vbatt[V],lf,ls,rs,rf,enc_r,enc_l,odo_dist[mm],odo_ang[rad],ball_raw,ball_det`
  (`software/mob/README.md` の SEN 記述と一致)。
- **引数なし `STOP` コマンドは存在しない**(`STOP,<v>,<a>,<d>` のみ)。
  無条件のモータ停止は `MOT,0,0`、減速停止は `QSTP`(`QSTPDONE,<残距離>` が返る)。
- `FWD` は距離到達で DONE を返すが**停止しない**(連続走行用)。停止するのは `STOP`。
- `STOP`/`TURN` の DONE は「動作完了かつ整定済み」(2026-07-24〜)。完了後
  0.5 秒の角度維持ホールド(v=0+角度FB)を済ませてから DONE が返るため、
  DONE 待ちに従来より最大+0.5秒かかる。DONE のタイミングを早める変更は
  慣性回転中の角度を次コマンドが基準角として取り込む問題を再発させるので不可。
- `WALL,<0|1>`(壁センサ LED)には DONE 応答が**ない**。
- `LFWD`/`LBACK`/`LTURNL`/`LTURNR`(LATCH系)・`LSTOP` も DONE 応答が**ない**
  送りっぱなしコマンド。`LSTOP` が来るまで低速固定速度(`params.latch_mps`/
  `latch_turn_mps`)で動き続ける「ボタン押下中だけ動かす」手動操作向け
  (ゲームパッド遠隔操作、`software/manual_controller/` 参照)。距離/角度
  指定で自動停止・DONE を返す `JOGFWD`/`JOGBACK`/`JOGTURN` とは用途が違う。
- `GCAL`/`RDST`/`RANG` は DONE を返す。TURN は正=左回り(CCW)、単位 rad。
- `SANG,<angle_rad>`(2026-07-24〜)は `RANG` の0固定版ではなく任意値版:
  ジャイロ積分角度を外部の絶対基準(カメラによる壁上面検出など、
  `software/micromouse/vision.py` 参照)で上書きする。DONE を返す。
  RANG/RDST 同様、**セグメント間の停止中にのみ**呼ぶこと(走行中の
  制御ループが参照する目標角には触れないので安全だが、動作中に呼ぶと
  次セグメントの基準角が汚染される)。
- `make upload` 直後の初回シリアル接続は SEN 応答を取りこぼしやすい
  (ポートオープン時の ESP32 自動リセットとの競合)。`sensor read failed` で
  落ちたら再実行すればよい。
- ⚠️ 2026-07-17 時点、**実機の ESP32 には旧ファーム**(SEN 7フィールド、
  FWD/STOP/TURN 非対応)が入っていた。走行系の作業前に
  `cd software/mob && make upload PORT=/dev/ttyUSB0` で要更新。
  arduino-cli は `~/bin/arduino-cli`。コンパイルのみなら `make build`(安全)。
- pyserial 無しの疎通確認(モータを回さない):
  `stty -F /dev/ttyUSB0 3000000 raw -echo` → `printf 'SEN\n' > /dev/ttyUSB0`
  → `timeout 1 stdbuf -o0 cat /dev/ttyUSB0`
  (読み取りを先に開始してから送信しないと応答を取りこぼす)
- エンコーダ:ホイールのギア比 `GEAR_RATIO = 1.0`(直結、2026-07-18 に
  41/20 から変更)は `sensors.h` と `motion_controller.cpp` に**重複定義**
  されている。変更時は両方を直すこと
  (片方だけだと走行距離とオドメトリがずれる)。

## venv・テストの規約

- 開発用 venv は `software/venv`(共有、`--system-site-packages`)。
  `beacon/venv` だけ独立。本番は `/usr/bin/python3` 直(venv 不使用)。
- **環境を変える操作(pip install、ファーム書き込み等)は勝手に実行しない**。
  requirements.txt への記載と手順の提示に留め、実行はユーザーに委ねる。
  (2026-07-17 時点で `software/venv` に pyserial は未導入。)
- テスト実行:
  ```bash
  software/venv/bin/python3 -m unittest discover -s software/<dir>/tests -q
  ```
  micromouse のテストは pyserial 無しでも全て走る(例外定義を `errors.py` に
  分離してあるため)。この構造は維持すること。
- 既知の既存テスト失敗: `software/ui/tests` の `test_sigterm_calls_cleanup`
  (`SystemExit` 未捕捉)。micromouse 実装とは無関係。

## 実機の systemd サービスの実態

- **インストール済み unit はリポジトリ内の `.service` ファイルと一致しない**。
  実態は必ず `systemctl cat <unit>` で確認すること。
- 例: `ui_server.service` の実機定義は `User=k-iida` で、ExecStart は
  `/opt/ui` ではなく**このリポジトリの作業ツリーを直接参照**
  (`software/venv/bin/python3 .../software/ui/ui_server.py`)。
  つまりコード修正の反映は cp 不要で `sudo systemctl restart ui_server` のみ。
  逆に言うと、**作業ツリーの未コミット変更がそのまま本番で動く**ので、
  master 上で編集中のコードはサービス再起動で即座に実機挙動に影響する。
- リポジトリ側 `ui/README.md` の `/opt/ui` デプロイ手順・`ui_server.service`
  は理想形/参考であり、実機の現状とは別物(2026-07-18 時点)。

## micromouse の設計要点(software/micromouse/)

- 実装一式は **2026-07-24 に master へマージ済み**(7416797、fast-forward)。
  走行チューニングも同日完了: 閉路パターン(8マス+旋回5回)で物理ずれ
  2mm・1°、壁センサFB有効(比例方式、左25mmずれを1.8マスで回収)。
  経緯は c816154/b94f658/cd86ad1/dc23386/7416797 のコミットメッセージ参照。
- 依存方向は「ハードウェア → センサ抽象 → 迷路アルゴリズム」。
  `maze.py`/`explorer.py`/`path_planner.py` は純 Python でハード非依存。
  `simulator.py` は `mobile_base.py` と同一インターフェース(ダックタイピング)。
- 座標系: (0,0) が左下、x=東、y=北。探索の判断点は**セル境界**
  (センサはそこで進入先セルの壁を読む)。セル走行は半セル 90mm×2(Twilight 実績)。
- 未知壁の扱い: 探索=通行可(楽観)、最短経路計画=壁(安全)。
- **スタートセルの壁は決め打ちしない**(Twilight は東壁ありと仮定していたが、
  実大会迷路には東が開いた面があり探索不能になる。スタートでもセンサ観測する)。
- 迷路ファイル(`maze_files/*.txt`)は mm_maze_solver 互換 ASCII。
  シミュレーションでの回帰確認:
  ```bash
  software/venv/bin/python3 software/micromouse/micromouse_app.py \
      --sim software/micromouse/maze_files/AllJapan_002_1981_classic___16x16.txt \
      --no-ui --autostart
  ```
- 実機の段階検証は `hw_test.py`(sen → walls → gcal → fwd → turn → cycle の順)。
- 壁しきい値(config/micromouse.yaml)はセンサ個体依存。実機で `hw_test.py walls`
  を見ながら校正する(未校正のまま実走行しない)。

## その他の運用メモ

- コミットメッセージは `<dir>: <日本語の要約>` 形式(git log 参照)。
- `default_app` へのアプリ追加はコード変更不要
  (`/etc/robot-ui/applications.yaml` に追記。default_ui.py がメニューの
  「Applications」選択のたびに読み直すのでサービス再起動も不要)。実機の
  同ファイルは root 所有で、リポジトリ作業ツリーを直接参照する形式
  (`software/venv/bin/python3` で `software/micromouse/micromouse_app.py` 等を
  起動)。2026-07-24 に Micromouse / Pattern Test を登録済み。YAML の
  `priority` フィールドは子プロセスへ渡らず装飾的(所有権は default_ui が
  子起動時に自ら disconnect して解放し、子アプリが自前の priority で
  ui_server に接続する)。リポジトリ側 `config/applications.yaml.example` は
  /opt デプロイ時の理想形の例。
- Discord Webhook 設定は 環境変数 `DISCORD_WEBHOOK_URL` → `beacon/.env` →
  `beacon/config.json` の優先順(camera/default_app/beacon で共通)。
  投稿処理を書くときは `beacon/discord_ip.py` の `load_webhook_url()` を
  import して再利用し、`{"content": ...}` を POST する(既存コードと同形式)。
