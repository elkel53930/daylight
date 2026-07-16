# `mob.ino` 解析

対象: `software/mob/mob.ino`（2026-07-16 時点、コミット `d607372`）

## 1. 概要

`mob.ino` は ESP32-S3 上で動く移動ロボット（daylight「mob」機体）のメインスケッチ。
PC（ホスト）から UART（3,000,000bps）でテキストコマンドを受け取り、モーター駆動・
オドメトリ・壁センサ追従・各種プロファイル走行（直進/停止/旋回/低速動作）を行う。

内部的には `robosweep_twilight`（同種の過去プロジェクト、コメント中の "T"）の制御ロジックを
daylight のハードウェア抽象層（"D"）に移植したもの。ソースコメントに散見される
"T→D" 表記はその移植の名残。

## 2. 全体アーキテクチャ

### 2コア構成（FreeRTOS on ESP32-S3）

| コア | タスク | 役割 |
|------|--------|------|
| Core0 | `Core0RealtimeTask`（最高優先度） | 1kHz 周期のリアルタイム制御ループ |
| Core1 | Arduino 標準 `loop()` | UARTコマンドの受信・解析、Core0からのメッセージ出力 |

2つのコア間は FreeRTOS のキューで通信する。

```
Core1 (loop)                         Core0 (Core0RealtimeTask, 1kHz)
  UART受信 → コマンド解析              cmd_queue から受信 → 状態機械を更新
  cmd_queue へ送信      ───────────►   センサ更新・モーション制御・PID
  msg_queue から受信 → Serial出力 ◄─── msg_queue へメッセージ送信
```

- `cmd_queue`（深さ16, `Command` 構造体）: Core1 → Core0 のコマンド伝達。
- `msg_queue`（深さ128, `MsgLine` 構造体, 63文字+終端）: Core0 → Core1 のログ／応答文字列伝達。
  Core0側では `Serial.print` を直接呼ばず、`enqueue_msg_line()` で溜めて Core1 側の
  `loop()` 冒頭でまとめて `Serial.print` する設計（"Serial.printfはCore1側でのみ行う"という
  コメントが `loop()` にある）。

### 1msタイマー

`hw_timer_t* high_speed_timer` を 1000Hz で駆動し、`onHighSpeedTimer()`（ISR）が
`std::atomic<uint32_t> timer_ticks` をインクリメントする。`Core0RealtimeTask` は
`waitTick()` でこのカウンタが進むまでビジーウェイト（`nop`）し、経過tick数（≒経過ms）を
`sensors.update()` や各モーション更新関数に `dt_s` として渡す。

## 3. 起動処理 `setup()`

1. `Serial.begin(3000000)`
2. `disableCore0WDT()` — Core0のハードウェアWDTを無効化（1msビジーウェイトループがWDTに
   引っかからないようにするため）
3. Wi-Fi/Bluetoothを明示的にOFF（`WiFi.mode(WIFI_OFF)`, `btStop()`）— ADC2やCPU資源を
   ペリフェラル側に確保するため（壁センサ左・バッテリー電圧はADC2を使用しWi-Fi非使用時のみ有効、
   READMEに明記）
4. 各ドライバの初期化: 共有SPI、`motor`、`encoder`、`imu`、`wall_sensor`、`battery`、`led`
5. `cmd_queue` / `msg_queue` の作成
6. `Core0RealtimeTask` を Core0 に最高優先度でピン留めして起動

## 4. グローバルなクラスインスタンス

```cpp
LED led;
WallSensor wall_sensor;
IMU imu(get_shared_spi());
Motor motor;
Encoder encoder;
Battery battery;
Sensors sensors(imu, wall_sensor, battery, encoder);
MotionController motion(motor, sensors);
```

`Sensors` が IMU/壁センサ/バッテリー/エンコーダを束ね、`MotionController` が
`Motor` と `Sensors` を使って左右輪速度PID制御を行う上位ラッパー。

## 5. コマンドプロトコル（UART, `\n` 終端, `\r\n` も許容）

| コマンド | 引数 | 説明 |
|----------|------|------|
| `MOT,<r>,<l>` | 右/左速度 [mm/s] | 手動速度指令（プロファイル走行は全キャンセル） |
| `WALL,<0\|1>` | 有効/無効 | 壁センサLEDの有効化（デフォルト無効） |
| `FWD,<speed_mmps>,<accel_mmps2>,<distance_mm>` | 速度・加速度・追加距離 | 加速して指定速度に達し、距離到達で完了通知（停止しない） |
| `STOP,<speed_mmps>,<accel_mmps2>,<distance_mm>` | 巡航速度・減速度・追加距離 | 指定距離で停止するよう減速（50mm/s進入→最終停止） |
| `TURN,<angle_rad>` | 相対回転角 [rad] | その場旋回（+:左回り） |
| `LFWD` / `LBACK` / `LTURNL` / `LTURNR` | なし | 低速の連続動作開始（`LSTOP`まで継続） |
| `LSTOP` | なし | 低速連続動作の停止 |
| `JOGFWD,<distance_mm>` / `JOGBACK,<distance_mm>` | 距離 | 低速(50mm/s)で指定距離だけ移動 |
| `RDST` | なし | 距離オドメトリ・累積目標距離をリセット |
| `RANG` | なし | 角度オドメトリをリセット |
| `GCAL` | なし | ジャイロオフセットの自動キャリブレーション（非ブロッキング、100サンプル平均） |
| `QSTP` | なし | 現在の速度から最大減速度でクイック停止 |
| `SEN` | なし | センサ一括値を1行で返す |

いずれのコマンドも `cmd_queue` へ `Command` 構造体（`union` で各コマンドのパラメータを
共用）として送信され、`Core0RealtimeTask` 側の `processCommandQueue()` で処理される。

完了通知は多くのコマンドで `"DONE\n"` が `msg_queue` 経由（一部 `Serial.printf` 直接）で
返る。`QSTP` のみ `"QSTPDONE,<remaining_dist>\n"` という別フォーマットで返す。

`SEN` 応答フォーマット:
```
SEN,<gyro_z rad/s>,<vbatt V>,<lf>,<ls>,<rs>,<rf>,<enc_r>,<enc_l>,<odo_dist mm>,<odo_angle rad>
```
（`lf`/`rf` は同じ前壁センサ値を指す。壁センサは前・左・右の3個のみで前壁を左右兼用として返している。）

## 6. モーション状態機械（`Core0RealtimeTask` 内、1kHz）

各ループで以下を順に実行:

1. `waitTick()` で次の1ms tickまで待機、経過時間 `dt_s` を取得
2. `sensors.update(time_delta)` — IMU/壁センサ/バッテリー/エンコーダを読み、
   オドメトリ（`distance_`, `angle_`）を積分更新
3. `processCommandQueue()` — Core1から届いたコマンドを処理し、対応する `*_active` フラグと
   目標値をセット（新しいコマンドが来ると、競合する他のプロファイルは全て非アクティブ化される）
4. 優先順位付きで状態を更新:
   ```
   if (!updateLatch(dt) && !updateJog(dt)) {
       if (!updateQstp(dt)) {
           updateForward(dt);
           updateStop(dt);
           updateTurn(dt);
       }
   }
   ```
   すなわち `LATCH`（L*系）> `JOG` > `QSTP` > (`FWD`/`STOP`/`TURN` は排他フラグで実質1つだけ動く)
   の優先度。
5. ジャイロキャリブレーション完了チェック
6. `motion.update(time_delta)` — 目標左右輪速度に対して速度PIDを実行し、モーターPWMへ反映

### 各プロファイルの挙動

- **FWD**（`updateForward`）: 目標速度まで指定加速度で加速/減速しながら直進。残距離が
  30mm以上ある間だけ壁センサによる横方向補正（`calculate_wall_correction`）を行い、
  常時ジャイロ角度・角速度フィードバックで直進方向を保つ。距離到達で速度指令はそのまま
  （停止せず）`DONE` を通知。
- **STOP**（`updateStop`）: 指定距離で止まるよう減速。残距離に応じて
  「巡航速度維持 → 50mm/s まで減速 → 20mm/s（最低速度）で最終進入 → 停止」の3フェーズ。
  4秒でタイムアウトすると、その場で停止した後 30mm 後退する「バックオフ」動作に入る
  （壁に接触してホイールが空転している場合などの保護と思われる）。
- **TURN**（`updateTurn`）: 目標角度との誤差にP制御をかけ、上下限速度でクランプ、
  slew-rate制限で滑らかに加減速しながら `motion.turn_in_place()` でその場旋回。
  誤差が約1.7°以内で完了。
- **LATCH**（`updateLatch`, `LFWD/LBACK/LTURNL/LTURNR/LSTOP`）: 距離/角度目標を持たない、
  `LSTOP` が来るまで続く低速の連続動作（前進/後退/旋回）。
- **JOG**（`updateJog`）: 開始時点からの移動距離（絶対値）を追跡し、指定距離（±2mm）に
  達したら停止して `DONE`。
- **QSTP**（`updateQstp`）: 実行中だった `FWD`/`STOP` の目標距離を退避した上で、
  最大減速度（1000mm/s²）で速度をゼロまで落とす。停止完了時、退避しておいた目標距離との
  差分を `QSTPDONE,<remaining_dist>` で返す（ホスト側が続きの動作を再開できるようにする
  ためと推測される）。

### 横方向補正のロジック（`calculate_wall_correction`）

左右の壁センサ値をそれぞれ目標値（機体固有のキャリブレーション定数、コード内に
「１号機」「２号機」の値が併記されコメントアウトで切り替え）と比較し、片側のみ有効
（閾値100以上）ならその側だけで補正、両方有効なら平均、両方無効なら補正なしとする。

### 角度/角速度フィードバック

直進系（FWD/STOP/QSTP）は共通して
`lateral_correction = ANGLE_FB_GAIN * angle_error + ANGULAR_RATE_FB_GAIN * gyro_z`
を `motion.forward(v, lateral_correction)` に渡し、`MotionController::forward()` 内で
左右輪目標速度に反映させる。

## 7. `MotionController` / `Sensors` との関係

- `Sensors` は Core0 の `update()` で全センサをまとめて読み、`std::atomic` 経由で
  Core1からも安全に読み出せるようにしている。オドメトリ（距離・角度）もここで積分。
- `MotionController` は `forward()`/`backward()`/`turn_in_place()`/`stop()` という
  高レベルAPIを提供し、内部でエンコーダ差分から左右輪の実速度を推定して
  PID（P=800, I=60, D=0）で目標速度に追従させ、`Motor::set_right/left()`
  （-1023〜+1023）に出力する。

## 8. ハードウェア対応（README.md より要約）

壁センサ3個（内蔵ADC）、AS5047エンコーダ×2（専用SPI/FSPI）、バッテリー電圧監視、
MCPWM group 0 によるモータードライバ、LSM6DSR IMU（専用SPI/HSPI）、LED（赤/緑）、
吸引ファン（LEDC PWM）、RCサーボ（MCPWM group 1）、ボールセンサ（内蔵ADC）。

ただし **ファン・サーボ・ボールセンサのドライバ（`fan.h/cpp`, `servo.h/cpp`,
`ball_sensor.h/cpp`）は `mob.ino` から一切参照されていない**（詳細は
`mob_review.md` を参照）。
