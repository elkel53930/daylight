# `mob.ino` コードレビュー

対象: `software/mob/mob.ino`（2026-07-16 時点、コミット `d607372`）。
`arduino-cli compile --fqbn esp32:esp32:esp32s3 mob.ino` で実際にビルドを試して検証。

## 重大: 現状ビルドが通らない

`arduino-cli compile` すると以下の4件のコンパイルエラーで失敗する。

```
mob.ino:147:1: error: 'LED' does not name a type; did you mean 'Led'?
mob.ino:149:9: error: 'get_shared_spi' was not declared in this scope
mob.ino:927:5: error: 'init_shared_spi' was not declared in this scope
mob.ino:933:5: error: 'led' was not declared in this scope; did you mean 'Led'?
```

原因は次の3点、いずれも `robosweep_twilight` からの移植時の命名ミスマッチと思われる。

1. **`LED` vs `Led`** — `mob.ino:147` の `LED led;` に対し、`led.h` で定義されているのは
   `class Led`（`led.h:10`）。`LED` という型は存在しない。
2. **`get_shared_spi()` / `init_shared_spi()` が存在しない** — `mob.ino:149`
   （`IMU imu(get_shared_spi());`）と `mob.ino:927`（`init_shared_spi();`）が呼んでいる
   関数名だが、`spi_manager.h/cpp` で実際に定義されているのは `get_imu_spi()` /
   `init_imu_spi()`（`spi_manager.h:9-10`）。

このスケッチは今のリポジトリの状態のままでは書き込みはおろかコンパイルすら通らない。
最優先の修正対象。

**修正案**: `mob.ino:147` を `Led led;` に、`mob.ino:149` を `IMU imu(get_imu_spi());` に、
`mob.ino:927` を `init_imu_spi();` に変更するだけで解消するはず（実際にこの3箇所を
直して再コンパイルすれば確認できる）。

## Core0（リアルタイムタスク）から直接 `Serial.printf` を呼んでいる箇所がある

`loop()`（Core1）の冒頭に以下のコメントで設計方針が明記されている:

> `// Core0からのメッセージ出力（Serial.printfはCore1側でのみ行う）`

これに従い、`updateForward`/`updateStop`/`updateTurn`/`handle*Command` はすべて
`enqueue_msg_line()` で `msg_queue` 経由にしているが、**`updateJog()`（`mob.ino:590`）だけ
`Serial.printf("DONE\n")` を直接呼んでいる**。

```cpp
if (remaining_mm <= 2.0f) {
    jog_active = false;
    ...
    Serial.printf("DONE\n");   // ← ここだけ Core0 から直接 Serial に書いている
    return true;
}
```

Core1側の `loop()` も同時に `msg_queue` から取り出した文字列を `Serial.print()` している
ため、`JOGFWD`/`JOGBACK` の完了タイミングによっては2つのコアが同時に `Serial`（実体は
`HardwareSerial`、内部バッファやFIFOアクセスにロックがない）へ書き込むことになり、
出力の破損・文字化けや、最悪 `HardwareSerial` 内部状態の競合を招く可能性がある。
他の完了通知と同様に `enqueue_msg_line("DONE\n")` に統一すべき。

## `MotionController::turn_in_place()` への呼び出し方が内部の完了判定と噛み合っていない

`mob.ino` 側の `updateTurn()`（`mob.ino:779-809`）は、毎ループ「現在角度から見た残り角度」
`err` を計算し、それを `target_rel` として **そのまま** `motion.turn_in_place(speed, target_rel)`
に渡している。

```cpp
const float err = turn_goal_angle_rad - now_ang;
...
const float target_rel = err; // 現在から見た残り角度
(void)motion.turn_in_place(turn_speed_cmd_mps, target_rel);
```

一方 `MotionController::turn_in_place()`（`motion_controller.cpp:49-75`）は、
初回呼び出し時の角度を `turn_start_angle_rad_` として固定し、以後は

```cpp
const float turned = current - turn_start_angle_rad_;
if ((target_angle_rad >= 0 && turned >= target_angle_rad) || ...) {
    stop();
    turn_active_ = false;
    return true;
}
```

というように、**その回の呼び出しで渡された `target_angle_rad`（=呼び出し毎に縮んでいく
残り角度）** と「内部基準からの累積回転量」を比較して完了判定・`stop()` を行っている。

`target_angle_rad` に固定目標ではなく「毎回縮んでいく残り角度」を渡しているため、
おおよそ残り角度が半分になったタイミングで `turned >= target_angle_rad` が真になり、
`MotionController` 内部で `stop()`（PID積分値リセット・モーター速度0出力）が呼ばれ、
`turn_active_` がリセットされる。しかし `mob.ino` 側の `turn_active` は生きたままなので
次のループでまた `turn_in_place()` が「新規旋回」として再初期化される。この結果、
1回の `TURN`/`LTURNL`/`LTURNR` の間に **残り角度が半分になるたびに内部で
勝手にモーター速度0・PID積分リセットが挟まる**（90°の旋回なら理論上6回前後、
45°,22.5°,11.25°,…と等比に減りながら発生）。

外部から見た完了判定は `mob.ino` 側の `TURN_DONE_TOL_RAD`（約1.7°）でしか行われず
実害（旋回が止まらない等）はないが、旋回中に周期的な速度カクつき・振動を起こしている
可能性が高い。実機で旋回動作にガタつきがある場合はここが疑わしい。

**修正の方向性**: `turn_in_place()` に渡す `target_angle_rad` は「旋回開始時の固定目標」に
すべきで、`mob.ino` 側で最初の1回だけ角度を確定し、以降は同じ値を渡し続けるようにする
（あるいは `MotionController` 側の完了判定自体を削除し、完了検知は呼び出し側の
`updateTurn()` に一本化する）。

## README.md がシリアルプロトコルの現状と一致していない

`README.md` の「シリアルコマンド」節は `MOT`, `WALL`, `FAN`, `SRV`, `BALL`, `SEN`,
単純な `STOP` のみを記載しているが、実際の `mob.ino` はこれと全く異なるコマンド体系
（`FWD`, `STOP,<speed>,<accel>,<dist>`, `TURN`, `LFWD/LBACK/LTURNL/LTURNR/LSTOP`,
`JOGFWD/JOGBACK`, `RDST`, `RANG`, `GCAL`, `QSTP`）を実装しており、`FAN`/`SRV`/`BALL`
コマンドは存在しない。`SEN` のレスポンス形式もREADME記載
（`ball_raw`/`ball_det` を含む9引数）と実装（`gyro,vbatt,lf,ls,rs,rf,enc_r,enc_l,odo_dist,odo_ang`
の10引数）で食い違っている。

このズレはロボット側の実装が正だと考えられる（`robosweep_twilight` 移植コミットで
コマンド体系を丸ごと差し替えたにもかかわらずREADMEを追随させていない）。
ホスト側ソフトウェアの実装者がREADMEだけを見ると誤ったプロトコルで実装してしまうため、
早めにREADMEを実装に合わせて更新すべき。

## `fan.h/cpp`, `servo.h/cpp`, `ball_sensor.h/cpp` が `mob.ino` から未使用

`README.md` はファン・サーボ・ボールセンサの配線とファイル一覧を明記しているが、
`mob.ino` は `Fan`/`Servo`/`BallSensor` のインクルードもインスタンス化も一切行っておらず、
`FAN`/`SRV`/`BALL` コマンドの受信処理も存在しない。ドライバ自体（`.cpp`）は実装済みで
コンパイル可能な状態にあるとみられるため、意図的に一旦外したのか、配線待ちで
未統合なのか、単なる統合漏れなのかが記述からは判別できない。恒久的に使わないなら
READMEから記述を外し、使う予定ならスケッチへの組み込みが必要。

## 軽微: コメントと定数値の不一致

`updateForward()` 内のコメントは

```cpp
// 壁センサフィードバック（残距離15mm未満ではオフ）
if (remain_mm >= WALL_CORRECTION_CUT_OFF_DISTANCE) { ... }
```

となっているが、実際の `WALL_CORRECTION_CUT_OFF_DISTANCE` は `30.0f`（`mob.ino:108`）。
コメントの「15mm」は古い値の書き残しと思われ、実際の閾値（30mm）と食い違っている。
実害はないが、チューニング時に誤解を招くため修正推奨。

## 軽微: 壁センサ目標値が号機ごとにコード直書き・コメントアウト切り替え

```cpp
// １号機
//static constexpr float WALL_SENSOR_TARGET_LS = 250.0f;
//static constexpr float WALL_SENSOR_TARGET_RS = 234.0f;
// ２号機
static constexpr float WALL_SENSOR_TARGET_LS = 233.0f;
static constexpr float WALL_SENSOR_TARGET_RS = 209.0f;
```

機体固有のキャリブレーション値をソースコードのコメントアウトで切り替える運用になっており、
別機体用にビルドし直す際にコメントの付け替え漏れが起きやすい。個体差のあるパラメータは
別ファイル（機体識別付きコンフィグ等）に切り出すか、せめてビルドフラグで切り替えられる
ようにしておくと事故が減る。

## まとめ（優先度順）

| # | 内容 | 深刻度 |
|---|------|--------|
| 1 | `LED`/`Led`・`get_shared_spi`/`get_imu_spi`・`init_shared_spi`/`init_imu_spi` の名前不一致でビルド不可 | **致命的（要即修正）** |
| 2 | `updateJog()` のみ Core0 から直接 `Serial.printf` している（マルチコア設計違反） | 中 |
| 3 | `turn_in_place()` への引数の渡し方が内部の完了判定ロジックと矛盾し、旋回中に不要な速度リセットが周期的に発生 | 中 |
| 4 | README.md のシリアルプロトコル記述が実装と全く一致していない | 中（ドキュメント負債） |
| 5 | `Fan`/`Servo`/`BallSensor` がドライバのみ存在しスケッチ未統合 | 低〜中（要意図確認） |
| 6 | コメント中の閾値（15mm）と実際の定数（30mm）の不一致 | 低 |
| 7 | 壁センサ目標値のコメントアウトによる号機切り替え | 低（保守性） |
