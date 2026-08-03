# CLAUDE.md — Daylight 開発知見

このリポジトリで作業する際の前提知識。2026-07-17〜18 の開発セッション
(マイクロマウス実装、ブザーのハードウェア PWM 化)で得た知見をまとめたもの。

⚠️ **2026-08-02、`software/mob/` の移動系制御(FWD/STOP/TURN/JOGFWD/
JOGBACK/JOGTURN/LFWD/LBACK/LTURNL/LTURNR/LSTOP/QSTP)と、それに依存していた
`software/micromouse/`・`software/manual_controller/` を丸ごと削除し、
その場旋回を中心に制御ロジックを根本から作り直し中**(モータードライバ・
IMU・エンコーダ等の低レベル層はそのまま)。以下のドキュメント中、削除済み
コマンド・ディレクトリへの言及は歴史的経緯として残しているが、実際には
存在しない。現状の唯一の移動系コマンドは `MOT`(速度制御)・`DUTY`(生duty)・
`HOLD`(その場静止制御、`place_controller.cpp`)のみ。
`micromouse_implementation_report.md`(リポジトリ直下)は削除前の実装の
歴史的記録として残っているが、コード自体はもう無い。

## リポジトリ概要

RoboSweep エージェント **Daylight**(Twilight の後継機)のリポジトリ。
開発・実行環境はロボット搭載の Raspberry Pi CM4 上(このリポジトリを clone
した場所がそのまま実機)。

```
elec/       KiCad 基板データ
software/
  default_app/  ホーム画面アプリ(systemd: default-ui.service、稼働中)
  ui/           OLED/ボタン/ブザーの UI サーバー(systemd: ui_server.service、稼働中)
  camera/       CSI カメラスクリプト(手動実行)
  beacon/       起動時 Discord IP 通知
  mob/          ESP32-S3 ファームウェア(Arduino/C++、モータ・センサ制御)
```

(`micromouse/`・`manual_controller/` は2026-08-02に削除。上記の注記参照)

参考実装(前身機): https://github.com/elkel53930/robosweep_twilight
— mob のシリアルプロトコルはほぼ共通。迷路アルゴリズム・走行ループの
設計思想の出典でもある。

## ハードウェア構成の要点

- **mob (ESP32-S3)** が全ての低レベル制御を持つ: 1kHz 制御ループ、車輪速度
  PID、ジャイロ角度/角速度 FB、壁センサ横補正、台形加速の距離走行。
  ラズパイはシリアルでコマンドを送るだけ。
- 接続: `/dev/ttyUSB0`(FTDI FT231X)、**3,000,000 bps**。
### 光壁センサ(前・左横・右横の3個、2026-08-03 ユーザー詳細指摘)

発した光の**反射強度**で壁までの距離を測る光センサ(LED ON/OFF 差分方式)。
SEN 応答の `lf`/`rf` は**どちらも前センサ値**(Twilight 4センサとの互換のため)。
前壁センサと横壁センサは**同一部品だが個体差がある**。重要な特性:

- **距離だけでなく角度の影響も受ける**。壁が光線に対して**90度でない(斜め)と
  値が小さく出る**。よって機体が迷路軸に正対していないと値は信頼できない。
- **前壁センサは前へまっすぐ**向いている。
- **横壁センサは真横より少し前向きに斜め**に向いている。このため:
  - 機体が進行方向に対して**少し右を向くと右センサ値が大きく・左が小さく**なる
    (逆も同様)。=向き(ヨー)の推定にも使えるが、横位置と結合する。
  - **前壁までの距離が近いと、横壁センサの光が前壁に反射して横壁センサ値が
    大きくなる**(前壁クロストーク)。角の手前では側壁値が過大に出る。
- **壁が近すぎると値が下がる(非単調)**。近距離域では距離が近いほど低下するため
  「近すぎ」と「遠い/開放」を値だけでは区別できない。**壁に近づきすぎた状況で
  センサを使ってはいけない**。近距離前壁の確実な検出はカメラで行う。
- **壁の色に個体差があり反射率がばらつく**ので、センサ値も多少ばらつく。
- 実測の目安(マス中心・正対時): 前壁90mm先で lf≈800、開放≈2、1.5セル先≈188。
  側壁は壁ありで ls/rs≈330(90mm)、開放≈10。ただし上記の角度・近接・色の
  影響で常にばらつくので、絶対しきい値に頼りすぎない。
- ⚠️ **横壁センサ利用時は壁に近づきすぎないこと。壁に近づく場合は横壁センサを
  無効化する**(WALL,0)。壁追従で差(rs-ls)を使うときは near-straight ゲート・
  近接ガード(近すぎ側から離れる)を必須にする。

### その他のセンサ・機構(2026-08-03 ユーザー詳細指摘)

- **ジャイロ(IMU LSM6DSR、gyro z のみ使用)**: Z軸がロボットの旋回方向。
  ドリフトするが**キャリブレーション(GCAL)で影響を低減**できる。**積分誤差が
  蓄積**する(=長時間・多数動作で角度がずれる。壁/カメラで随時再基準化する)。
- **ロータリーエンコーダ(AS5047、14bit 0–16383)**: 磁気式でホイールの回転を
  直接計測。ホイールとモーターは**平歯車**でつながっており**バックラッシがある**
  (微小往復や停止直後の位置は遊びぶん不定になりうる)。
- UI: OLED 96×64 + ボタン L/R + ブザー。`ui_server` に Unix ドメインソケットで
  接続(4バイト長プレフィックス + MessagePack)。優先度制御あり
  (default_app は priority=100 で最低、アプリは 20 前後で奪取)。
- ブザー(GPIO 13)は BCM2711 ハードウェア PWM(PWM0 ch1、sysfs 経由)駆動
  (2026-07-18〜)。config.txt に `dtoverlay=pwm,pin=13,func=4` 設定済み。
  ⚠️ **lgpio 等で GPIO 13 を出力として claim しないこと** — ALT0 マックスが
  解除され、PWM 波形がピンに出なくなる(無音。復旧は `pinctrl set 13 a0`)。
  sysfs 権限は udev が export 後に非同期付与するため直後の書き込みは
  EACCES になり得る。詳細は `software/ui/README.md`。

### 搭載カメラ(Picamera2 + Futabaサーボ、2026-08-03 ユーザー詳細指摘)

- カメラは **Futabaサーボ**を介して搭載。**サーボ0度でちょうど正面**を向く。
- **画像を使うときは必ずサーボ角度を0度**にすること。
- **サーボ角度の再現性にクセ**がある: 一度**20度ぐらいへ動かしてから0度へ戻す**と
  再現性が向上する(0度へ持っていく前にこのワンクッションを入れる)。
- ⚠️ **サーボ角度を0度未満に設定してはいけない**(ハードウェア故障の可能性)。
- **走行中に画像を取得してもブレてエッジ検出精度が出ない**。撮影は必ず静止時に。
- フォーカス・ホワイトバランスは**自動調整**(露出安定を待ってから使う)。
- ⚠️ **カメラは唯一、壁との位置・角度を「絶対値」として得られるセンサ**
  (ロボットのコース内座標=何マス目かが既知である前提)。**位置が大きくずれた
  ときはカメラを使ってマス中心へ戻す**こと。gyro/エンコーダは相対・積分で
  ドリフトするので、絶対基準としてはカメラを使う。

### 俯瞰カメラ(x13u の C270、2026-08-03 ユーザー詳細指摘)

- 解像度は **1280x960(Quad-VGA、HDではない)**。HD(1280x720)として撮ると
  迷路の一部が欠けるので注意。撮影は 1280x960 で行う。
- 映っている迷路(コース)は **4x4マス**。**1マスは縦横180mm**。俯瞰から
  ロボット位置を微調整する際はこの寸法を基準にする。
- カメラは迷路の**ほぼ真上**に設置。中心の4マスはほぼ真上から見下ろす構図で
  **視線が斜めでないためマス内のロボット位置を取得しやすい**(外周マスは斜め)。
- ⚠️ **ロボットを動かした(旋回・移動)ら、画像を VGA サイズに縮小して Discord へ
  投稿する**こと。

## 開発の進め方のルール(2026-08-03 ユーザー指示、必ず守る)

- **ロボットを動かすときは、動かす1秒ほど前にブザーを鳴らして周りに知らせる**。
- **走行のたびに位置・速度・角度・角速度などのログを確認し、発振などが起きて
  いないか必ず確認しながら進める**。振動が続くとハードウェア故障や制御精度低下に
  つながる。
- **横壁センサ利用時は壁に近づきすぎない。壁に近づく場合は横壁センサを無効化**する。
- ロボットは迷路サイズに対してそれなりに小さく、**位置が5mm程度ずれても深刻な
  問題は起きない**。ただし**5mmを超えて大きくならないよう制御を工夫**し、**5mmを
  超えそうだと検出できるとなお良い**。
- **位置が大きくずれたらカメラ(絶対基準)でマス中心へ戻す**(座標既知の前提)。

## mob シリアルプロトコルの落とし穴

- 現行 `mob.ino` の SEN 応答は **12 フィールド**(2026-07-24〜、ボールセンサ拡張):
  `SEN,gyro[rad/s],vbatt[V],lf,ls,rs,rf,enc_r,enc_l,odo_dist[mm],odo_ang[rad],ball_raw,ball_det`
  (`software/mob/README.md` の SEN 記述と一致)。
- 2026-08-02、距離・角度プロファイルを持つ移動コマンド(FWD/STOP/TURN/
  JOGFWD/JOGBACK/JOGTURN/LFWD/LBACK/LTURNL/LTURNR/LSTOP/QSTP)を全て削除し、
  制御を作り直し中。現状残っている移動系コマンドは以下のみ:
  - `MOT,<r>,<l>`: 左右輪に目標速度[mm/s]を即座に設定(車輪速度PID経由、
    `motion_controller.cpp`)。距離・完了通知の概念はない。
  - `DUTY,<r>,<l>`: 速度PIDを経由しない生duty直接指令(−1023〜+1023)。
  - `HOLD`: その場静止制御を開始(`place_controller.cpp`)。左右輪速度の
    和(並進成分)をエンコーダFBでゼロへ追い込む。180°その場旋回を
    作り込むための第一歩として2026-08-02に新規追加。停止は`MOT,0,0`。
  - `TURN,<angle_rad>`(正=左/CCW): その場旋回(2026-08-02〜)。目標角度を
    角加速度一定の台形速度プロファイルで滑らかに変化させ追従する。
    `HOLD`と同じ並進ゼロ制御を同時に動かしたまま行うため、旋回中に
    機体の位置がずれるのを防ぐ設計。旧`TURN`(角度のみのその場旋回、
    2026-08-02に削除)とはコマンド名が同じだけで実装は別物
    (`place_controller.cpp`ベースに刷新)。停止は`MOT,0,0`。
  - いずれも DONE 応答は返さない(送りっぱなし、または`HOLD`/`TURN`のように
    完了概念が無い継続動作。目標角度に達した後もそのまま角度保持を
    続ける)。
- `WALL,<0|1>`(壁センサ LED)にも DONE 応答が**ない**。
- `GCAL`(ジャイロキャリブレーション)・`RDST`(距離リセット)・`RANG`
  (角度リセット)は DONE を返す。
- `SANG,<angle_rad>` は `RANG` の0固定版ではなく任意値版: ジャイロ積分角度を
  外部の絶対基準で上書きする。DONE を返す。RANG/RDST 同様、走行中の制御
  ループが参照する目標角には触れないため安全だが、意味を持たせたい場合は
  機体が静止しているときに呼ぶこと。
  ⚠️ **`RANG`/`SANG` を `TURN` の角度保持中(place_controller が動作中)に
  出してはいけない**(2026-08-03判明)。TURN の追従は
  `angle_error = turn_goal - get_angle()` を使うが、RANG/SANG は `get_angle()`
  (=odo_ang)だけを書き換え `turn_goal` は旧フレームのまま残る。すると
  angle_error が突然大きくなり、機体が旧ゴールを追って暴れ回る(実機で
  コースを一周して隅まで移動)。**必ず先に `MOT,0,0`(stop)で TURN 保持を
  抜けてから** RANG/SANG を出すこと。`GCAL` も同様、静止時に呼ぶ。
- `make upload` 直後の初回シリアル接続は SEN 応答を取りこぼしやすい
  (ポートオープン時の ESP32 自動リセットとの競合)。`sensor read failed` で
  落ちたら再実行すればよい。
- pyserial 無しの疎通確認(モータを回さない):
  `stty -F /dev/ttyUSB0 3000000 raw -echo` → `printf 'SEN\n' > /dev/ttyUSB0`
  → `timeout 1 stdbuf -o0 cat /dev/ttyUSB0`
  (読み取りを先に開始してから送信しないと応答を取りこぼす)
- エンコーダ:ホイールのギア比 `GEAR_RATIO = 1.0`(直結)は `sensors.h`・
  `motion_controller.cpp`・`place_controller.cpp` に**重複定義**されている。
  変更時は3箇所とも直すこと(片方だけだと走行距離とオドメトリがずれる)。

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
- 既知の既存テスト失敗: `software/ui/tests` の `test_sigterm_calls_cleanup`
  (`SystemExit` 未捕捉)。

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

## micromouse・manual_controller(削除済み、2026-08-02)

`software/micromouse/`(マイクロマウス自律走行)と `software/manual_controller/`
(ゲームパッド遠隔操作)は、依存していた mob の移動系コマンド(FWD/STOP/TURN/
JOG系/LATCH系/QSTP)ごと2026-08-02に削除した。ESP32側の制御ロジックを
「その場に静止する」→「180°その場旋回」から根本的に作り直すため
(`software/mob/README.md`のコマンド一覧・`place_controller.cpp`参照)。

- 設計・チューニングの詳細な経緯は `micromouse_implementation_report.md`
  (リポジトリ直下)と git 履歴(`git log -- software/micromouse
  software/manual_controller`)に残っている。作り直す際の参考にすること。
- 未追跡(gitでは管理されていない)の実機データが `software/micromouse/`
  配下にまだ残っている: `config/micromouse.yaml`(壁センサしきい値の実機
  校正値)、`logs/`(走行・パターンテストのログ、カメラ較正データ含む、
  約12MB)。コードは削除したがこれらは削除していない
  (再校正の手間を省くため)。
- `default_app` のメニュー(`/etc/robot-ui/applications.yaml`、実機・root
  所有)から Micromouse / Pattern Test / Manual Control のエントリは削除済み。
  Camera Test のみ残っている。

## その他の運用メモ

- コミットメッセージは `<dir>: <日本語の要約>` 形式(git log 参照)。
- `default_app` へのアプリ追加はコード変更不要
  (`/etc/robot-ui/applications.yaml` に追記。default_ui.py がメニューの
  「Applications」選択のたびに読み直すのでサービス再起動も不要)。実機の
  同ファイルは root 所有で、リポジトリ作業ツリーを直接参照する形式
  (`software/venv/bin/python3` でリポジトリ内スクリプトを起動)。現状は
  Camera Test のみ登録(2026-08-02、Micromouse/Pattern Test/Manual Control
  は依存スクリプトの削除に伴い削除)。YAML の `priority` フィールドは
  子プロセスへ渡らず装飾的(所有権は default_ui が子起動時に自ら
  disconnect して解放し、子アプリが自前の priority で ui_server に
  接続する)。リポジトリ側 `config/applications.yaml.example` は /opt
  デプロイ時の理想形の例。
- Discord Webhook 設定は 環境変数 `DISCORD_WEBHOOK_URL` → `beacon/.env` →
  `beacon/config.json` の優先順(camera/default_app/beacon で共通)。
  投稿処理を書くときは `beacon/discord_ip.py` の `load_webhook_url()` を
  import して再利用し、`{"content": ...}` を POST する(既存コードと同形式)。
