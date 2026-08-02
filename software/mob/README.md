# mob

ESP32-S3 上で動作する移動ロボット制御ファームウェア。

## ハードウェア構成

### 光センサ（壁センサ）3個 — 内蔵ADC使用

| センサ | EN ピン | アナログ入力ピン |
|--------|---------|----------------|
| 右     | IO4     | IO5            |
| 前     | IO6     | IO7            |
| 左     | IO15    | IO16           |

LED の ON/OFF 差分を取ることで環境光をキャンセルしています。

### エンコーダ（AS5047） — FSPI

| 信号   | ピン  |
|--------|-------|
| R CS   | IO46  |
| L CS   | IO9   |
| SCK    | IO10  |
| MISO   | IO11  |
| MOSI   | IO12  |

14bit 分解能（0–16383）。

### バッテリー電圧監視

| 信号         | ピン  |
|--------------|-------|
| アナログ入力 | IO13  |

IO13 の電圧 × 11 = バッテリー電圧。

### モータードライバ — MCPWM group 0

| 信号     | ピン  |
|----------|-------|
| PWM L    | IO21  |
| CWCCW L  | IO47  |
| PWM R    | IO48  |
| CWCCW R  | IO45  |

PWM 周波数: ~39kHz（40MHz / 1024 ticks）、速度指定: −1023〜+1023。

### IMU（LSM6DSR） — HSPI

| 信号 | ピン  |
|------|-------|
| CS   | IO41  |
| SCK  | IO40  |
| MOSI | IO39  |
| MISO | IO38  |

ジャイロ FS: ±1000 dps、ODR: 416 Hz。

### LED

| 色   | ピン  | 用途                           |
|------|-------|--------------------------------|
| 緑   | IO20  | シリアル受信ごとに赤と交互点滅 |
| 赤   | IO3   | シリアル受信ごとに緑と交互点滅 |

### 吸引ファン — LEDC

| 信号      | ピン  |
|-----------|-------|
| FET ゲート | IO2  |

NchFET ゲート駆動。PWM 周波数: 40kHz、8bit 分解能（0–255）。

### RCサーボ（MG90S） — MCPWM group 1

| 信号 | ピン |
|------|------|
| SIG  | IO1  |

PWM 周波数: 50Hz、パルス幅: 500–2500µs（0–180°）。

### ボールセンサ（光反射式）

| 信号         | ピン  |
|--------------|-------|
| アナログ入力 | IO14  |

12bit ADC（0–4095）。しきい値以上で「ボール検出」と判定（デフォルトしきい値: 2048）。

## ファイル構成

| ファイル | 説明 |
|----------|------|
| `mob.ino` | メインスケッチ |
| `wall_sensor.h/cpp` | 壁センサドライバ（内蔵ADC） |
| `battery.h/cpp` | バッテリー電圧監視 |
| `encoder.h/cpp` | AS5047 エンコーダドライバ（FSPI） |
| `imu.h/cpp` | LSM6DSR IMU ドライバ（HSPI） |
| `motor.h/cpp` | モータードライバ（MCPWM group 0） |
| `led.h/cpp` | LED ドライバ（IO20: 緑、IO3: 赤） |
| `fan.h/cpp` | 吸引ファン PWM ドライバ（LEDC、IO2） |
| `servo.h/cpp` | RC サーボドライバ（MCPWM group 1、IO1） |
| `ball_sensor.h/cpp` | ボールセンサドライバ（内蔵ADC、IO14） |
| `motion_controller.h/cpp` | 車輪速度PID・フィードフォワード（MOT/DUTYコマンド用） |
| `place_controller.h/cpp` | その場静止制御（左右輪速度の和をゼロへ、`HOLD`コマンド用、2026-08-02〜） |
| `params.h/cpp` | 機体固有チューニングパラメータ（NVS永続化、PGET/PSET等） |
| `spi_manager.h/cpp` | IMU 用 HSPI バス管理 |
| `Makefile` | ビルド / 書き込み / モニタ |

## ビルド・書き込み

```bash
# コンパイルのみ（差分ビルド）
make

# コンパイル＋書き込み
make upload

# ポートを指定する場合
make upload PORT=/dev/ttyACM0

# シリアルモニタ（3Mbps）
make monitor

# 差分ビルドキャッシュ削除
make clean

# 全キャッシュ削除（フルビルドに戻す）
make clean-all
```

ビルド成果物は `.build/` に保存され、変更ファイルのみ再コンパイルされます。

### リモートビルド（任意）

`make build`（`make upload` 経由も含む）は、リモートビルドサーバーが
設定・到達可能ならそちらでコンパイルし、成果物だけを持ち帰る。未設定・
到達不可・リモート側のビルド失敗（arduino-cli 未導入等）の場合は自動的に
Raspberry Pi 上でのローカルビルドにフォールバックするため、設定しなくても
今まで通り動く。設計の詳細は `/remote_build.md` を参照。

有効にするには:

```bash
cp Makefile.local.example Makefile.local
# Makefile.local を編集して BUILD_HOST を設定
```

`Makefile.local` は `.gitignore` 対象（ホスト名・ユーザー名を含むため）。
ビルドサーバー側にも同じ FQBN の Arduino Core（`arduino-cli core install
esp32:esp32`）が必要。ビルドサーバー側の `arduino-cli` が PATH に無い場合
（SSH の非対話シェルでは `.bashrc` の PATH 追加が反映されないため）は
`scripts/arduino-build.sh` 内のパスを実際の設置場所に合わせて修正する。

⚠️ ローカルとリモートで Arduino Core のバージョンが異なると、生成される
バイナリも変わりうる（実測: 3.3.10 と 3.3.11 でサイズが約1.4%異なった）。
両ビルド環境のコアバージョンは揃えておくこと。

## シリアルコマンド（3,000,000 bps）

コマンドはエンター（`\n`）で確定します。`\r\n` にも対応しています。

| コマンド | 説明 |
|----------|------|
| `MOT,<r>,<l>` | 右/左モーター速度設定（−1023〜+1023） |
| `DUTY,<r>,<l>` | 右/左モーターへ生duty直接指令（−1023〜+1023、速度PID非経由。校正・診断用） |
| `WALL,<0\|1>` | 壁センサ LED 有効/無効 |
| `FAN,<0-255>` | 吸引ファン速度設定 |
| `SRV,<0-180>` | サーボ角度設定（度） |
| `SRVOFF` | サーボのトルクオフ（脱力） |
| `BALL,<0-4095>` | ボールセンサしきい値設定 |
| `PGET` | 全パラメータ一覧取得(`PVAL,<name>,<value>` を1行ずつ、最後に `PLISTEND`) |
| `PGET,<name>` | 単一パラメータ取得(`PVAL,<name>,<value>`) |
| `PSET,<name>,<value>` | パラメータ即時変更(RAM上のみ、NVSには保存しない) |
| `PSAVE` | 現在のパラメータを丸ごとNVSへ保存(機体固有の恒久設定) |
| `PLOAD` | NVSから読み込みRAMへ反映(起動時にも自動実行) |
| `PRESET` | RAM上のパラメータをビルド時デフォルトへ戻す(NVSは変更しない) |
| `SEN` | センサデータ一括取得 |
| `HOLD` | その場静止制御を開始(左右輪速度の和をゼロへ、`place_controller.cpp`)。停止は`MOT,0,0` |

### パラメータ(PGET/PSET/PSAVE/PLOAD/PRESET)

旋回PID・壁センサ基準値など機体差の出やすいチューニング値は
`params.h`/`params.cpp` の `Params` 構造体にまとめてあり、ビルドし直さず
シリアル経由で調整・機体ごとに恒久保存できる。

```
PGET                        # 一覧
PGET,angle_fb_gain          # 単一取得
PSET,angle_fb_gain,0.5      # RAM上で即時変更(次のモーションから反映)
PSAVE                       # 現在のRAM値をNVS(内蔵フラッシュ)へ保存
PLOAD                       # NVSから再読込(起動時にも自動実行)
PRESET                      # RAM値をビルド時デフォルトへ戻す(NVSは無変更)
```

パラメータはパラメータ名ごとに個別のNVSキーで保存されるため、将来
`Params` にフィールドを追加/削除しても他の調整済み値には影響しない
(新フィールドは保存キーが無いのでデフォルトのまま動く)。
一覧・各パラメータの意味は `params.h` のコメント参照。

### SEN レスポンス形式

```
SEN,<gyro_z>,<vbatt>,<lf>,<ls>,<rs>,<rf>,<enc_r>,<enc_l>,<odo_dist>,<odo_ang>,<ball_raw>,<ball_det>
```

| フィールド | 型 | 説明 |
|---|---|---|
| `gyro_z` | float | Z軸角速度 [rad/s] |
| `vbatt` | float | バッテリー電圧 [V] |
| `lf/ls/rs/rf` | uint16 | 壁センサ差分値（`lf`/`rf`は共に前センサ値、Twilight 4センサとの互換のため） |
| `enc_r/l` | uint16 | エンコーダ角度 0–16383（右/左） |
| `odo_dist` | float | オドメトリ走行距離 [mm] |
| `odo_ang` | float | オドメトリ角度 [rad] |
| `ball_raw` | uint16 | ボールセンサ ADC 生値 0–4095 |
| `ball_det` | 0\|1 | ボール検出フラグ（1=検出） |
