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

### モータードライバ

| 信号     | ピン  |
|----------|-------|
| PWM L    | IO21  |
| CWCCW L  | IO47  |
| PWM R    | IO48  |
| CWCCW R  | IO45  |

PWM 周波数: 40kHz / 10bit 分解能（速度指定: −1023〜+1023）。

### IMU（LSM6DSR） — HSPI

| 信号 | ピン  |
|------|-------|
| CS   | IO41  |
| SCK  | IO40  |
| MOSI | IO39  |
| MISO | IO38  |

ジャイロ FS: ±1000 dps、ODR: 416 Hz。

## ファイル構成

| ファイル | 説明 |
|----------|------|
| `mob.ino` | メインスケッチ |
| `wall_sensor.h/cpp` | 光センサドライバ（内蔵ADC） |
| `battery.h/cpp` | バッテリー電圧監視 |
| `encoder.h/cpp` | AS5047 エンコーダドライバ（FSPI） |
| `imu.h/cpp` | LSM6DSR IMU ドライバ（HSPI） |
| `motor.h/cpp` | モータードライバ（40kHz PWM） |
| `spi_manager.h/cpp` | IMU 用 HSPI バス管理 |
| `Makefile` | ビルド / 書き込み / モニタ |

## ビルド・書き込み

```bash
# コンパイルのみ
make

# コンパイル＋書き込み
make upload

# ポートを指定する場合
make upload PORT=/dev/ttyACM0

# シリアルモニタ（3Mbps）
make monitor

# キャッシュ削除
make clean
```

## シリアルコマンド（3,000,000 bps）

| コマンド | 説明 |
|----------|------|
| `MOT,<r>,<l>` | 右/左モーター速度設定（−1023〜+1023） |
| `WALL,<0\|1>` | 壁センサ LED 有効/無効 |
| `SEN` | センサデータ一括取得 |
| `STOP` | モーター停止 |

### SEN レスポンス形式

```
SEN,<gyro_z rad/s>,<batt V>,<wall_r>,<wall_f>,<wall_l>,<enc_r>,<enc_l>
```
