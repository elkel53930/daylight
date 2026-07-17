# CLAUDE.md — Daylight 開発知見

このリポジトリで作業する際の前提知識。2026-07-17 のマイクロマウス機能実装
セッションで得た知見をまとめたもの。詳細な経緯は
`micromouse_implementation_report.md` を参照。

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

## mob シリアルプロトコルの落とし穴

- 現行 `mob.ino` の SEN 応答は **11 フィールド**:
  `SEN,gyro[rad/s],vbatt[V],lf,ls,rs,rf,enc_r,enc_l,odo_dist[mm],odo_ang[rad]`
  — `software/mob/README.md` の SEN 記述は古い(9フィールド)ので注意。
- **引数なし `STOP` コマンドは存在しない**(`STOP,<v>,<a>,<d>` のみ)。
  無条件のモータ停止は `MOT,0,0`、減速停止は `QSTP`(`QSTPDONE,<残距離>` が返る)。
- `FWD` は距離到達で DONE を返すが**停止しない**(連続走行用)。停止するのは `STOP`。
- `WALL,<0|1>`(壁センサ LED)には DONE 応答が**ない**。
- `GCAL`/`RDST`/`RANG` は DONE を返す。TURN は正=左回り(CCW)、単位 rad。
- ⚠️ 2026-07-17 時点、**実機の ESP32 には旧ファーム**(SEN 7フィールド、
  FWD/STOP/TURN 非対応)が入っていた。走行系の作業前に
  `cd software/mob && make upload PORT=/dev/ttyUSB0` で要更新。
  arduino-cli は `~/bin/arduino-cli`。コンパイルのみなら `make build`(安全)。
- pyserial 無しの疎通確認(モータを回さない):
  `stty -F /dev/ttyUSB0 3000000 raw -echo` → `printf 'SEN\n' > /dev/ttyUSB0`
  → `timeout 1 stdbuf -o0 cat /dev/ttyUSB0`

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

## micromouse の設計要点(software/micromouse/)

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
  (`/etc/robot-ui/applications.yaml` に追記。実機にはまだこのファイルが無い)。
- Discord Webhook 設定は 環境変数 `DISCORD_WEBHOOK_URL` → `beacon/.env` →
  `beacon/config.json` の優先順(camera/default_app/beacon で共通)。
