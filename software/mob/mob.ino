#include <SPI.h>
#include <WiFi.h>
#include <atomic>
#include <esp_task_wdt.h>
#include "spi_manager.h"
#include "led.h"
#include "wall_sensor.h"
#include "battery.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "sensors.h"
#include "motion_controller.h"
#include "fan.h"
#include "servo.h"
#include "ball_sensor.h"
#include <math.h>

// Target wheel speed [m/s] (updated from MOT command via cmd_queue)
static float target_vr_mps = 0.0f;
static float target_vl_mps = 0.0f;

// 前進コマンド（プロファイル走行）状態
static bool fwd_active = false;
static float fwd_v_cmd_mmps = 0.0f;     // 現在指令速度 [mm/s]
static float fwd_v_target_mmps = 0.0f;  // 目標巡航速度 [mm/s]
static float fwd_a_mmps2 = 0.0f;        // 加速度 [mm/s^2]（符号付き）
static float fwd_goal_dist_mm = 0.0f;   // 絶対目標距離 [mm]（Sensors::get_distance()基準）
static float fwd_target_angle_rad = 0.0f; // 目標角度 [rad] （角度フィードバック用）
static float cumulative_goal_dist_mm = 0.0f; // 累積目標距離 [mm]（RDSTでリセット）

// 停止コマンド（減速して停止する）状態
static bool stop_active = false;
static float stop_v_cmd_mmps = 0.0f;      // 現在指令速度 [mm/s]
static float stop_a_mmps2 = 0.0f;         // 減速度 [mm/s^2]（負ではなく絶対値として扱う）
static float stop_goal_dist_mm = 0.0f;    // 絶対目標距離 [mm]
static float stop_target_angle_rad = 0.0f; // 目標角度 [rad] （角度フィードバック用）
static float stop_cruise_mmps = 0.0f;     // STOP引数speed_mmps（巡航速度想定）[mm/s]
static float stop_elapsed_s = 0.0f;       // STOP経過時間 [s]
static bool stop_backoff_active = false;  // タイムアウト後の後退中か
static float stop_backoff_target_dist_mm = 0.0f; // 後退完了目標距離 [mm]

static constexpr float FINAL_APPROACH_SPEED_MMPS = 50.0f;  // STOP時の最終進入速度
static constexpr float STOP_MIN_SPEED_MMPS = 40.0f;        // STOP時の最低速度 [mm/s]
                                                           // 20だと静止摩擦に負けて目標手前でスタックする(2026-07-18実機)
static constexpr float STOP_TIMEOUT_SEC = 4.0f;            // STOPのタイムアウト [s]
static constexpr float STOP_BACKOFF_DIST_MM = 30.0f;       // タイムアウト時の後退距離 [mm]
static constexpr float STOP_BACKOFF_SPEED_MPS = 0.12f;     // タイムアウト時の後退速度 [m/s]

// 旋回コマンド（その場旋回）状態
static bool turn_active = false;
static float turn_goal_angle_rad = 0.0f;
static float turn_speed_cmd_mps = 0.0f; // 指令速度（加減速制限後）
static bool turn_settling = false;      // 整定確認フェーズ中
static float turn_settle_elapsed_s = 0.0f;
static uint8_t turn_retry_count = 0;    // 再サーボ回数
static float turn_integ = 0.0f;         // PID積分項 [rad*s]（リトライ含め1回のTURNコマンド内で保持）

// 低速動作コマンド（L*）状態
enum class LatchMode : uint8_t {
    NONE = 0,
    FWD,
    BACK,
    TURN_L,
    TURN_R,
};

static bool latch_active = false;
static LatchMode latch_mode = LatchMode::NONE;
static float latch_turn_target_rad = 0.0f;

static constexpr float LATCH_SPEED_MPS = 0.05f;        // 50mm/s（Lowspeed）
static constexpr float LATCH_TURN_SPEED_MPS = 0.06f;   // 約90deg/s相当（Lowspeed）
static constexpr float LATCH_TURN_TARGET_RAD = 1000000.0f; // 低速連続旋回の仮想目標角

// JOGコマンド（距離指定低速動作）状態
static bool jog_active = false;
static bool jog_is_forward = true;  // true: 前進, false: 後退
static float jog_target_dist_mm = 0.0f;
static float jog_start_dist_mm = 0.0f;
static constexpr float JOG_SPEED_MPS = 0.05f;  // 50mm/s

// ジャイロキャリブレーション状態
static bool gyro_calib_done_pending = false;  // キャリブレーション完了時にDONEを返す

// QSTPコマンド（クイック停止）状態
static bool qstp_active = false;
static float qstp_v_cmd_mmps = 0.0f;      // 現在指令速度 [mm/s]
static float qstp_target_angle_rad = 0.0f; // 目標角度 [rad] （角度フィードバック用）
static float qstp_original_goal_dist_mm = 0.0f; // 元の目標距離 [mm]（FWD/STOPの目標）
static constexpr float QSTP_DECEL_MMPS2 = 1000.0f;  // QSTP時の最大減速度 [mm/s^2]

// 角度制御パラメータ（PID + 速度制限）
// P単独では減速が間に合わず、目標付近で毎回7〜15°程度オーバーシュートして
// 整定待ち(TURN_SETTLE_SEC)→再サーボのリトライを消費していた(2026-07-22
// 実機 Pattern Test ログで確認)。
//
// 第1版(I・D常時有効)では90°旋回は改善した(7〜15°→1.5〜3.7°)が、
// 180°旋回(TURN_BACK)は悪化した(-17.6°)。残角が大きい間(gz最大
// 8.7rad/s)積分項がアンチワインドアップ上限に張り付き続けて減速開始を
// 遅らせていたと考え、Iを条件付き積分(残角小のみ)にしたところ、
// 今度は複数の旋回でオーバーシュート・振動がさらに悪化した
// (例: 180°で-20.7°、7°旋回で3回リトライ)。原因はDにあった:
// TURN_KD_MPS_PER_RADPS×gz は巡航中の高い gz(6〜8rad/s)だけで
// 単独で約0.4m/sにも達し、Pがまだ十分な速度を要求している最中でも
// cmdを最低速度まで落としてしまい、減速タイミングを乱していた
// (残角0.7rad=40°でもcmdが60まで低下、実機ログで確認)。
//
// このため I・D いずれも残角が TURN_FINE_ENABLE_RAD 以下(=Pだけで
// 既に最低速度域に入っている、目標間際の最終アプローチ)のときのみ
// 有効化する「条件付きPID」にした。巡航中はPのみの素直な減速
// プロファイルを保ち、最終アプローチでだけI(残留誤差解消)と
// D(オーバーシュート抑制)を効かせる狙い。KI/KDは初期値であり、
// 実機の Pattern Test ログ(#V の ang/gz 推移)で要再チューニング。
static constexpr float TURN_KP_MPS_PER_RAD = 0.35f;      // [m/s]/rad
static constexpr float TURN_KI_MPS_PER_RAD_S = 0.3f;     // [m/s]/(rad*s)
static constexpr float TURN_KD_MPS_PER_RADPS = 0.05f;    // [m/s]/(rad/s)（実測角速度=gyro_zに掛ける）
static constexpr float TURN_FINE_ENABLE_RAD = 0.2f;     // 条件付き積分: |err|がこれ以下の間だけ積分を加算
// オーバーシュートの根本原因(整定判定の瞬間にduty=0にして
// TURN_SETTLE_SEC=80ms無制動で待つため、その時点のgzがそのまま飛び出し
// 量になる)を踏まえ、減速レート(TURN_ACCEL_MPS2)側のチューニングでは
// 方向によって効きムラが大きく限界だったため、そもそもの最高角速度
// 自体を下げて飛び出す物理量を減らす方針に変更(2026-07-22)。
// 0.25→0.15でオーバーシュート3.6〜6.6°・最終誤差0.9〜1.3°まで改善
// (リトライも2〜3回→1回に減少、2026-07-22実機ログで確認)。
// 0.12まで下げると右90°/180°はさらに改善したが、左90°は逆に悪化
// (4.6°→7.7°、リトライ1→2回)。左右差は速度を下げるだけでは解消しない
// (方向依存の非対称性がある)ため、0.15に戻して左右速度同期の検証に
// 切り替える。
static constexpr float TURN_MAX_SPEED_MPS = 0.15f;
static constexpr float TURN_MIN_SPEED_MPS = 0.06f;  // 0.04では終端が遅すぎた(2026-07-19)
static constexpr float TURN_DONE_TOL_RAD = 0.01f;     // 約0.57deg
static constexpr float TURN_SETTLE_SEC = 0.08f;       // 整定確認の待ち時間 [s]
static constexpr uint8_t TURN_MAX_RETRY = 3;          // 整定不足時の再サーボ最大回数
// 整定判定(|err|<=TURN_DONE_TOL_RAD)の瞬間にmotion.stop()でduty=0にして
// TURN_SETTLE_SEC待つが、その時点でgzがまだ8〜10rad/s残っていると
// 無制動の80ms慣性だけで10〜24°も飛び出すことが実機ログで確認された
// (2026-07-22)。1.2m/s²では巡航速度(250mm/s)から最低速度(60mm/s)まで
// 絞り切る前に整定判定に突入してしまうため、大幅に引き上げる。
// 1.2→4.0でオーバーシュートが大幅減少(90°で最大24°→14°、180°で
// 13°→4.5°、旋回によっては0°)。試しに8.0まで上げたところ、左90°旋回は
// 引き続き0°だったが右90°(14°→17°)と180°(4.5°→14.4°)はむしろ悪化した
// ため、4.0に戻す(単純に強いほど良いわけではなく、方向によって最適値が
// 違う可能性がある。左右差は wheel sync 無効化と合わせて要調査)。
static constexpr float TURN_ACCEL_MPS2 = 4.0f;        // 旋回時の車輪速度加速度制限 [m/s^2]

// 直進時の角度フィードバックゲイン
static constexpr float ANGLE_FB_GAIN = 1.0f;  // [m/s]/rad

// 直進時の角速度フィードバックゲイン（角速度→0）
static constexpr float ANGULAR_RATE_FB_GAIN = 0.3f;  // [m/s]/(rad/s)
                                                      // ヨー振動(GZ±0.3-0.6rad/s)対策で0.02から増強(2026-07-19)

// 壁センサフィードバックパラメータ
static constexpr float WALL_SENSOR_THRESHOLD = 100.0f;  // 壁検出閾値
// １号機
//static constexpr float WALL_SENSOR_TARGET_LS = 250.0f;     // 中央時の目標値
//static constexpr float WALL_SENSOR_TARGET_RS = 234.0f;     // 中央時の目標値
// ２号機(2026-07-19 セル中央実測 ls=299-322 / rs=216-242 の平均)
static constexpr float WALL_SENSOR_TARGET_LS = 311.0f;     // 中央時の目標値
static constexpr float WALL_SENSOR_TARGET_RS = 229.0f;     // 中央時の目標値

static constexpr float WALL_SENSOR_GAIN = 0.00000f;      // 壁センサフィードバックゲイン [m/s] per sensor unit
                                                          // 1kHzで目標角度に積算されるため実質積分器。0.000005では
                                                          // 目標角の移動が速すぎ角度ループと干渉して振動した(2026-07-19)
static constexpr float WALL_CORRECTION_CUT_OFF_DISTANCE = 30.0f; // 壁センサ補正を行う残距離の閾値 [mm]

// 壁センサを使用した横方向補正値を計算
// 戻り値: 正の値は右に寄せる補正、負の値は左に寄せる補正
static float calculate_wall_correction(const Sensors& sensors_ref) {
    const uint16_t rs_val = sensors_ref.get_rs();  // 右側センサ
    const uint16_t ls_val = sensors_ref.get_ls();  // 左側センサ
    
    const bool rs_valid = (rs_val >= WALL_SENSOR_THRESHOLD);
    const bool ls_valid = (ls_val >= WALL_SENSOR_THRESHOLD);
    
    float correction = 0.0f;
    
    if (rs_valid && ls_valid) {
        // 両方のセンサが有効な場合：両方を使用
        const float rs_error = WALL_SENSOR_TARGET_RS - static_cast<float>(rs_val);
        const float ls_error = static_cast<float>(ls_val) - WALL_SENSOR_TARGET_LS;
        correction = (rs_error + ls_error) * 0.5f * WALL_SENSOR_GAIN;
    } else if (ls_valid) {
        // 左センサのみ有効：左センサを使用して右に寄せる
        const float ls_error = static_cast<float>(ls_val) - WALL_SENSOR_TARGET_LS;
        correction = ls_error * WALL_SENSOR_GAIN;
    } else if (rs_valid) {
        // 右センサのみ有効：右センサを使用して左に寄せる
        const float rs_error = WALL_SENSOR_TARGET_RS - static_cast<float>(rs_val);
        correction = rs_error * WALL_SENSOR_GAIN;
    }
    // 両方無効な場合は correction = 0.0f のまま
    
    return correction;
}

static inline float slew_rate_limit(float current, float target, float max_delta) {
    if (target > current + max_delta) return current + max_delta;
    if (target < current - max_delta) return current - max_delta;
    return target;
}

// グローバルなクラスは、上から順に初期化される
Led led;
WallSensor wall_sensor;
IMU imu(get_imu_spi());
Motor motor;
Encoder encoder;
Battery battery;
Sensors sensors(imu, wall_sensor, battery, encoder);
MotionController motion(motor, sensors);
Fan fan;
Servo servo;
BallSensor ball_sensor;

hw_timer_t* high_speed_timer = NULL;
std::atomic<uint32_t> timer_ticks(0);

QueueHandle_t cmd_queue; // コマンドキュー
QueueHandle_t msg_queue; // Core0 -> Core1 メッセージキュー（Serial出力用）

// ---- Core1(loop) -> Core0(RealtimeTask) コマンド定義 ----
struct SetMotorSpeedCommand {
    int16_t right_speed; // mm/s（互換）
    int16_t left_speed;  // mm/s（互換）
};

struct ForwardCommand {
    float speed_mmps;
    float accel_mmps2;
    float distance_mm;
};

struct StopCommand {
    float speed_mmps;
    float accel_mmps2;
    float distance_mm;
};

struct LatchCommand {
    uint8_t mode; // LatchMode
};

struct JogCommand {
    bool is_forward;  // true: 前進, false: 後退
    float distance_mm;
};

enum CommandID : uint8_t {
    CMD_SET_MOTOR_SPEED = 0x01,
    CMD_FORWARD = 0x04,
    CMD_STOP = 0x05,
    CMD_RESET_DISTANCE = 0x06,
    CMD_RESET_ANGLE = 0x07,
    CMD_TURN = 0x08,
    CMD_GYRO_CALIBRATE = 0x09,
    CMD_LATCH_START = 0x0A,
    CMD_LATCH_STOP = 0x0B,
    CMD_JOG_START = 0x0C,
    CMD_QSTP = 0x0D,
};

struct Command {
    CommandID cmd_id;
    union {
        SetMotorSpeedCommand set_motor_speed;
        ForwardCommand forward;
        StopCommand stop;
        float turn_target_rad;
        LatchCommand latch;
        JogCommand jog;
    } parameter;
};

// ---- Core0 -> Core1 メッセージ定義（Serial出力用） ----
// Core0->Core1で渡すメッセージ（固定長; 末尾は必ず\0終端）
struct MsgLine {
    char text[64];
};

static inline void enqueue_msg_line(const char* s) {
    if (!msg_queue || !s) return;
    MsgLine m;
    size_t i = 0;
    bool truncated = false;
    for (; i < sizeof(m.text) - 1 && s[i] != '\0'; ++i) {
        m.text[i] = s[i];
    }
    // メッセージが長すぎる場合の検出
    if (s[i] != '\0' && i == sizeof(m.text) - 1) {
        truncated = true;
    }
    m.text[i] = '\0';
    // DONEなど重要なメッセージを確実に送るため、最大10msまで待つ
    (void)xQueueSend(msg_queue, &m, pdMS_TO_TICKS(10));
    
    // 切り捨てが発生した場合は警告を別途送信
    if (truncated) {
        MsgLine warn;
        snprintf(warn.text, sizeof(warn.text), "#WARN: msg truncated\n");
        (void)xQueueSend(msg_queue, &warn, 0);
    }
}

void IRAM_ATTR onHighSpeedTimer() {
    timer_ticks.fetch_add(1, std::memory_order_relaxed);
}

uint32_t waitTick(uint32_t &last_tick) {
    uint32_t current_tick = timer_ticks.load(std::memory_order_relaxed);
    while(current_tick == last_tick) {
        // 待機
        asm volatile("nop");
        current_tick = timer_ticks.load(std::memory_order_relaxed);
    }
    uint32_t delta = current_tick - last_tick;
    last_tick = current_tick;
    return delta;
}

// コマンド処理・モーション更新関数の前方宣言
void handleSetMotorSpeedCommand(const SetMotorSpeedCommand& cmd);
void handleForwardCommand(const ForwardCommand& cmd);
void handleStopCommand(const StopCommand& cmd);
void handleTurnCommand(float turn_target_rad);
void handleQstpCommand();
void processCommandQueue();
bool updateLatch(float dt_s);
bool updateJog(float dt_s);
void updateForward(float dt_s);
void updateStop(float dt_s);
void updateTurn(float dt_s);
bool updateQstp(float dt_s);

// ========================================
// コマンド処理関数の実装
// ========================================

void handleSetMotorSpeedCommand(const SetMotorSpeedCommand& cmd) {
    const float vr = static_cast<float>(cmd.right_speed) / 1000.0f;
    const float vl = static_cast<float>(cmd.left_speed) / 1000.0f;

    // 手動MOTが来たらプロファイルは停止（競合回避）
    fwd_active = false;
    stop_active = false;
    turn_active = false;
    latch_active = false;
    latch_mode = LatchMode::NONE;
    jog_active = false;
    qstp_active = false;
    stop_backoff_active = false;
    stop_elapsed_s = 0.0f;

    target_vr_mps = vr;
    target_vl_mps = vl;
    motion.forward((vr + vl) * 0.5f, 0.0f);
}

void handleForwardCommand(const ForwardCommand& cmd) {
    // FWD: 加速して指定速度へ、距離到達でDONE（停止しない）
    led.red_on();  // FWD開始時に赤色LED点灯
    fwd_active = true;
    stop_active = false;
    turn_active = false;
    latch_active = false;
    latch_mode = LatchMode::NONE;
    jog_active = false;
    qstp_active = false;
    stop_backoff_active = false;
    stop_elapsed_s = 0.0f;

    fwd_v_target_mmps = cmd.speed_mmps;
    fwd_a_mmps2 = cmd.accel_mmps2;

    // 累積目標距離に追加（オーバーシュートの影響を受けない）
    cumulative_goal_dist_mm += cmd.distance_mm;
    fwd_goal_dist_mm = cumulative_goal_dist_mm;
    
    // 目標角度を現在の角度に設定（まっすぐ進む）
    fwd_target_angle_rad = sensors.get_angle();

    // FWDコマンドの詳細情報を通知
    const float now_dist = sensors.get_distance();
    char msg[64];
    snprintf(msg, sizeof(msg), "#FWD: %.2f+%.2f->%.2f\n", now_dist, cmd.distance_mm, fwd_goal_dist_mm);
    enqueue_msg_line(msg);

    // 現在の指令速度（左右平均）を初期値に
    fwd_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;

    // 目標が今より遅いなら減速方向に
    if (fwd_v_target_mmps < fwd_v_cmd_mmps && fwd_a_mmps2 > 0) {
        fwd_a_mmps2 = -fwd_a_mmps2;
    }

    if (fwd_a_mmps2 == 0.0f) {
        fwd_v_cmd_mmps = fwd_v_target_mmps;
    }
}

void handleStopCommand(const StopCommand& cmd) {
    // STOP: 指定距離で停止（必要に応じて50mm/sまで減速して進入）
    stop_active = true;
    fwd_active = false;
    turn_active = false;
    latch_active = false;
    latch_mode = LatchMode::NONE;
    jog_active = false;
    stop_backoff_active = false;
    stop_elapsed_s = 0.0f;
    stop_backoff_target_dist_mm = 0.0f;

    // 引数の速度は「現在速度の想定」だが、ここでは現在の指令速度も併用
    stop_a_mmps2 = fabsf(cmd.accel_mmps2);
    stop_cruise_mmps = fabsf(cmd.speed_mmps);

    // 累積目標距離に追加（オーバーシュートの影響を受けない）
    cumulative_goal_dist_mm += cmd.distance_mm;
    stop_goal_dist_mm = cumulative_goal_dist_mm;
    
    // 目標角度を現在の角度に設定（まっすぐ進む）
    stop_target_angle_rad = sensors.get_angle();
    
    // STOPコマンドの詳細情報を通知
    const float now_dist = sensors.get_distance();
    char msg[64];
    snprintf(msg, sizeof(msg), "#STOP: %.2f+%.2f->%.2f\n", now_dist, cmd.distance_mm, stop_goal_dist_mm);
    enqueue_msg_line(msg);

    // 初期速度: 現在指令速度を優先
    stop_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;
    if (stop_v_cmd_mmps <= 0.0f) {
        stop_v_cmd_mmps = stop_cruise_mmps;
    }
}

void handleQstpCommand() {
    // QSTP: クイック停止（現在の速度から最大減速度で停止）
    
    // 元の目標距離を記録（FWDまたはSTOPが実行中の場合）
    qstp_original_goal_dist_mm = 0.0f;  // デフォルト値
    if (fwd_active) {
        qstp_original_goal_dist_mm = fwd_goal_dist_mm;
    } else if (stop_active) {
        qstp_original_goal_dist_mm = stop_goal_dist_mm;
    }
    
    qstp_active = true;
    fwd_active = false;
    stop_active = false;
    turn_active = false;
    latch_active = false;
    latch_mode = LatchMode::NONE;
    jog_active = false;
    stop_backoff_active = false;
    stop_elapsed_s = 0.0f;
    
    // 現在の指令速度を取得
    qstp_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;
    if (qstp_v_cmd_mmps < 0.0f) {
        qstp_v_cmd_mmps = -qstp_v_cmd_mmps;  // 絶対値にする
    }
    
    // 目標角度を現在の角度に設定（まっすぐ停止する）
    qstp_target_angle_rad = sensors.get_angle();
    
    char msg[64];
    snprintf(msg, sizeof(msg), "#QSTP: from %.1fmm/s\n", qstp_v_cmd_mmps);
    enqueue_msg_line(msg);
}

void handleTurnCommand(float target_rad) {
    // TURN: その場旋回（角度のみ）
    turn_active = true;

    // 競合回避: ほかのプロファイルを停止
    fwd_active = false;
    stop_active = false;
    latch_active = false;
    latch_mode = LatchMode::NONE;
    jog_active = false;
    qstp_active = false;
    stop_backoff_active = false;
    stop_elapsed_s = 0.0f;

    // 角度制御の基準を確定
    const float turn_start_angle_rad = sensors.get_angle();
    turn_goal_angle_rad = turn_start_angle_rad + target_rad;
    turn_speed_cmd_mps = 0.0f;
    turn_settling = false;
    turn_settle_elapsed_s = 0.0f;
    turn_retry_count = 0;
    turn_integ = 0.0f;

    // TURNコマンドの詳細情報を通知
    char msg[64];
    snprintf(msg, sizeof(msg), "#TURN_GOAL: %.6f\n", turn_goal_angle_rad);
    enqueue_msg_line(msg);

    // 回り始めは MotionController の内部turn状態を新規にするため stop() しておく
    motion.stop();
}

void processCommandQueue() {
    Command q;
    while (xQueueReceive(cmd_queue, &q, 0) == pdTRUE) {
        switch (q.cmd_id) {
            case CMD_SET_MOTOR_SPEED:
                handleSetMotorSpeedCommand(q.parameter.set_motor_speed);
                break;

            case CMD_FORWARD:
                handleForwardCommand(q.parameter.forward);
                break;

            case CMD_STOP:
                handleStopCommand(q.parameter.stop);
                break;

            case CMD_RESET_DISTANCE:
                sensors.reset_distance();
                cumulative_goal_dist_mm = 0.0f;  // 累積目標距離もリセット
                enqueue_msg_line("#distance reset\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_RESET_ANGLE:
                sensors.reset_angle();
                enqueue_msg_line("#angle reset\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_TURN:
                handleTurnCommand(q.parameter.turn_target_rad);
                break;

            case CMD_GYRO_CALIBRATE:
                // ジャイロキャリブレーション開始（非ブロッキング）
                enqueue_msg_line("#Gyro calibration start...\n");
                sensors.calibrate_gyro();
                gyro_calib_done_pending = true;
                break;

            case CMD_QSTP:
                handleQstpCommand();
                break;

            case CMD_LATCH_START: {
                latch_active = true;
                latch_mode = static_cast<LatchMode>(q.parameter.latch.mode);
                latch_turn_target_rad = 0.0f;

                // 競合回避: ほかのプロファイルを停止
                fwd_active = false;
                stop_active = false;
                turn_active = false;
                qstp_active = false;
                stop_backoff_active = false;
                stop_elapsed_s = 0.0f;

                if (latch_mode == LatchMode::TURN_L || latch_mode == LatchMode::TURN_R) {
                    latch_turn_target_rad = (latch_mode == LatchMode::TURN_L)
                        ? +LATCH_TURN_TARGET_RAD
                        : -LATCH_TURN_TARGET_RAD;
                    motion.stop();
                }
                break;
            }

            case CMD_LATCH_STOP:
                latch_active = false;
                latch_mode = LatchMode::NONE;
                latch_turn_target_rad = 0.0f;
                qstp_active = false;
                target_vr_mps = 0.0f;
                target_vl_mps = 0.0f;
                motion.stop();
                break;

            case CMD_JOG_START:
                jog_active = true;
                jog_is_forward = q.parameter.jog.is_forward;
                jog_target_dist_mm = q.parameter.jog.distance_mm;
                jog_start_dist_mm = sensors.get_distance();

                // 競合回避: ほかのプロファイルを停止
                fwd_active = false;
                stop_active = false;
                turn_active = false;
                latch_active = false;
                latch_mode = LatchMode::NONE;
                qstp_active = false;
                stop_backoff_active = false;
                stop_elapsed_s = 0.0f;
                break;

            default:
                break;
        }
    }
}

// ========================================
// モーション状態更新関数の実装
// ========================================

bool updateLatch(float dt_s) {
    (void)dt_s;
    if (!latch_active) return false;

    switch (latch_mode) {
        case LatchMode::FWD:
            target_vr_mps = LATCH_SPEED_MPS;
            target_vl_mps = LATCH_SPEED_MPS;
            motion.forward(LATCH_SPEED_MPS, 0.0f);
            return true;

        case LatchMode::BACK:
            target_vr_mps = -LATCH_SPEED_MPS;
            target_vl_mps = -LATCH_SPEED_MPS;
            motion.backward(LATCH_SPEED_MPS);
            return true;

        case LatchMode::TURN_L:
        case LatchMode::TURN_R: {
            const float s = LATCH_TURN_SPEED_MPS;
            motion.turn_in_place(s, latch_turn_target_rad);
            target_vr_mps = (latch_mode == LatchMode::TURN_L) ? +s : -s;
            target_vl_mps = (latch_mode == LatchMode::TURN_L) ? -s : +s;
            return true;
        }

        default:
            latch_active = false;
            latch_mode = LatchMode::NONE;
            return false;
    }
}

bool updateJog(float dt_s) {
    (void)dt_s;
    if (!jog_active) return false;

    const float current_dist_mm = sensors.get_distance();
    const float traveled_mm = fabsf(current_dist_mm - jog_start_dist_mm);  // 絶対値を取る
    const float remaining_mm = jog_target_dist_mm - traveled_mm;

    // 目標距離到達判定（±2mm許容）
    if (remaining_mm <= 2.0f) {
        jog_active = false;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
        enqueue_msg_line("DONE\n");
        return true;
    }

    // 低速で前進または後退
    if (jog_is_forward) {
        target_vr_mps = JOG_SPEED_MPS;
        target_vl_mps = JOG_SPEED_MPS;
        motion.forward(JOG_SPEED_MPS, 0.0f);
    } else {
        target_vr_mps = -JOG_SPEED_MPS;
        target_vl_mps = -JOG_SPEED_MPS;
        motion.backward(JOG_SPEED_MPS);
    }

    return true;
}

void updateForward(float dt_s) {
    if (!fwd_active) return;

    const float now_dist = sensors.get_distance();
    const float remain_mm = fwd_goal_dist_mm - now_dist;

    if (remain_mm <= 0.0f) {
        fwd_active = false;
        led.red_off();  // FWD完了時に赤色LED消灯
        enqueue_msg_line("DONE\n");
    } else {
        float v_next = fwd_v_cmd_mmps;
        const float a_mag = fabsf(fwd_a_mmps2);

        if (a_mag > 1e-3f) {
            if (fwd_v_target_mmps > fwd_v_cmd_mmps) {
                v_next = fwd_v_cmd_mmps + a_mag * dt_s;
                if (v_next > fwd_v_target_mmps) v_next = fwd_v_target_mmps;
            } else if (fwd_v_target_mmps < fwd_v_cmd_mmps) {
                v_next = fwd_v_cmd_mmps - a_mag * dt_s;
                if (v_next < fwd_v_target_mmps) v_next = fwd_v_target_mmps;
            }
        } else {
            v_next = fwd_v_target_mmps;
        }

        fwd_v_cmd_mmps = v_next;
        const float v_cmd_mps = fwd_v_cmd_mmps / 1000.0f;

        // 壁センサフィードバック（残距離15mm未満ではオフ）
        float wall_correction = 0.0f;
        if (remain_mm >= WALL_CORRECTION_CUT_OFF_DISTANCE) {
            wall_correction = calculate_wall_correction(sensors);
        }
        
        // 角度フィードバック: 現在角度と目標角度の差分
        fwd_target_angle_rad -= wall_correction;
        const float angle_error = sensors.get_angle() - fwd_target_angle_rad;
        const float angle_correction = ANGLE_FB_GAIN * angle_error;

        // 角速度フィードバック: 角速度をゼロに保つ
        const float gyro_z = sensors.get_gyro_z();
        const float rate_correction = ANGULAR_RATE_FB_GAIN * gyro_z;
        

        {
            static int dbg_count = 0;
            dbg_count++;
            if (dbg_count >= 50) {  // 20Hz
                dbg_count = 0;
                // MsgLine は64バイト上限のためCSV形式で短縮
                // #V,cmd,vr,vl,ur,ul,gz,ang,rem
                char msg[64];
                snprintf(msg, sizeof(msg),
                         "#V,%.0f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.1f\n",
                         fwd_v_cmd_mmps,
                         motion.get_vr_filt_mps() * 1000.0f,
                         motion.get_vl_filt_mps() * 1000.0f,
                         motion.get_duty_r(), motion.get_duty_l(),
                         gyro_z, sensors.get_angle(), remain_mm);
                enqueue_msg_line(msg);
            }
        }
        
        // 合計補正値
        const float lateral_correction = angle_correction + rate_correction;
        
        target_vr_mps = v_cmd_mps;
        target_vl_mps = v_cmd_mps;
        motion.forward(v_cmd_mps, lateral_correction);
    }
}

void updateStop(float dt_s) {
    if (stop_backoff_active) {
        const float now_dist = sensors.get_distance();
        if (now_dist <= stop_backoff_target_dist_mm) {
            stop_backoff_active = false;
            target_vr_mps = 0.0f;
            target_vl_mps = 0.0f;
            motion.stop();
            enqueue_msg_line("DONE\n");
        } else {
            target_vr_mps = -STOP_BACKOFF_SPEED_MPS;
            target_vl_mps = -STOP_BACKOFF_SPEED_MPS;
            motion.backward(STOP_BACKOFF_SPEED_MPS);
        }
        return;
    }

    if (!stop_active) return;

    stop_elapsed_s += dt_s;
    if (stop_elapsed_s >= STOP_TIMEOUT_SEC) {
        stop_active = false;
        stop_v_cmd_mmps = 0.0f;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
        stop_backoff_active = true;
        stop_backoff_target_dist_mm = sensors.get_distance() - STOP_BACKOFF_DIST_MM;
        return;
    }

    const float now_dist = sensors.get_distance();
    const float remain_mm = stop_goal_dist_mm - now_dist;

    // 目標距離に到達したら、まず停止指令を出し、次のループでDONEを返す
    if (remain_mm <= 0.0f) {
        stop_active = false;
        stop_v_cmd_mmps = 0.0f;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
        stop_elapsed_s = 0.0f;
        stop_backoff_active = false;
        enqueue_msg_line("DONE\n");
    } else {
        const float a_mag = stop_a_mmps2;
        const float v = stop_v_cmd_mmps;

        // まず 50mm/s まで減速する必要があるかを判断
        float dist_to_50 = 0.0f;
        if (a_mag > 1e-3f) {
            const float dv2 = (v * v) - (FINAL_APPROACH_SPEED_MMPS * FINAL_APPROACH_SPEED_MMPS);
            dist_to_50 = dv2 > 0.0f ? (dv2 / (2.0f * a_mag)) : 0.0f;
        }

        // 次に 50mm/s から 0 まで止めるのに必要な距離
        float dist_50_to_0 = 0.0f;
        if (a_mag > 1e-3f) {
            dist_50_to_0 = (FINAL_APPROACH_SPEED_MMPS * FINAL_APPROACH_SPEED_MMPS) / (2.0f * a_mag);
        }

        float v_next = v;

        // 残距離が「50->0の距離」以下なら、優先して停止に向けて減速（ただし20mm/s未満にはしない）
        if (remain_mm <= dist_50_to_0) {
            if (a_mag > 1e-3f) v_next = v - a_mag * dt_s;
            if (v_next < STOP_MIN_SPEED_MMPS) v_next = STOP_MIN_SPEED_MMPS;
        } else if (remain_mm <= (dist_to_50 + dist_50_to_0)) {
            // そろそろ 50mm/s まで落とすフェーズ
            if (a_mag > 1e-3f) v_next = v - a_mag * dt_s;
            if (v_next < FINAL_APPROACH_SPEED_MMPS) v_next = FINAL_APPROACH_SPEED_MMPS;
        } else {
            // まだ余裕がある: 今の速度維持（必要なら指定速度へ合わせる）
            // STOPの引数 speed_mmps は「巡航速度想定」なので、現在がそれ以下なら軽く合わせる
            const float cruise = stop_cruise_mmps;
            if (cruise > 0.0f && v < cruise && a_mag > 1e-3f) {
                v_next = v + a_mag * dt_s;
                if (v_next > cruise) v_next = cruise;
            }
        }

        stop_v_cmd_mmps = v_next;
        const float v_cmd_mps = stop_v_cmd_mmps / 1000.0f;

        // 角度フィードバック: 現在角度と目標角度の差分
        const float angle_error = sensors.get_angle() - stop_target_angle_rad;
        const float angle_correction = ANGLE_FB_GAIN * angle_error;

        // 角速度フィードバック: 角速度をゼロに保つ
        const float gyro_z = sensors.get_gyro_z();
        const float rate_correction = ANGULAR_RATE_FB_GAIN * gyro_z;

        // 合計補正値
        const float lateral_correction = angle_correction + rate_correction; // STOPでは壁センサ補正は行わない

        {
            static int dbg_count = 0;
            dbg_count++;
            if (dbg_count >= 50) {  // 20Hz
                dbg_count = 0;
                // #V,cmd,vr,vl,ur,ul,gz,ang,rem
                char msg[64];
                snprintf(msg, sizeof(msg),
                         "#V,%.0f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.1f\n",
                         stop_v_cmd_mmps,
                         motion.get_vr_filt_mps() * 1000.0f,
                         motion.get_vl_filt_mps() * 1000.0f,
                         motion.get_duty_r(), motion.get_duty_l(),
                         gyro_z, sensors.get_angle(), remain_mm);
                enqueue_msg_line(msg);
            }
        }

        target_vr_mps = v_cmd_mps;
        target_vl_mps = v_cmd_mps;
        if (v_cmd_mps == 0.0f) {
            motion.stop();
        } else {
            motion.forward(v_cmd_mps, lateral_correction);
        }
    }
}

void updateTurn(float dt_s) {
    if (!turn_active) return;

    const float now_ang = sensors.get_angle();
    const float err = turn_goal_angle_rad - now_ang;   // +: 左回り目標が残っている

    // 整定確認フェーズ: 一旦停止して待ち、慣性で流れた分を再確認する
    if (turn_settling) {
        turn_settle_elapsed_s += dt_s;
        if (turn_settle_elapsed_s < TURN_SETTLE_SEC) return;
        if (fabsf(err) <= TURN_DONE_TOL_RAD || turn_retry_count >= TURN_MAX_RETRY) {
            turn_active = false;
            turn_settling = false;
            enqueue_msg_line("DONE\n");
        } else {
            // 許容外に流れた: 再サーボ
            turn_settling = false;
            turn_retry_count++;
            turn_speed_cmd_mps = 0.0f;
        }
        return;
    }

    if (fabsf(err) <= TURN_DONE_TOL_RAD) {
        turn_settling = true;
        turn_settle_elapsed_s = 0.0f;
        turn_speed_cmd_mps = 0.0f;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
    } else {
        // PID制御で「理想速度」を作る。
        // P: 残角に比例(大きいほど速く)。
        // I: 整定待ち後もリトライ上限で許容差を超えたまま終わる残留誤差を解消。
        // D: 実測角速度(gyro_z)へのフィードバックで、目標接近時の
        //    ブレーキ不足によるオーバーシュートを抑える
        //    (err = goal - now_ang なので d(err)/dt = -gyro_z)。
        //
        // I・Dとも TURN_FINE_ENABLE_RAD 以下(最終アプローチ)でのみ有効化
        // する条件付き制御にしている。理由: 巡航中(残角が大きい間)は
        // gzが6〜8rad/s程度まで達するため、D単独で
        // TURN_KD_MPS_PER_RADPS×gz ≈ 0.4m/s もの制動項になり、Pがまだ
        // 十分な速度を要求している最中でもcmdが最低速度まで落ちてしまう
        // (2026-07-22 実機ログで確認: 残角0.7rad=40°でもcmdが60まで低下)。
        // これが減速タイミングを乱し、無条件Dにしたところ180°旋回で
        // オーバーシュートが悪化(-17.6°→-20.7°)した原因と考えられる。
        // 残角が小さい最終アプローチでのみDを効かせることで、巡航中は
        // Pだけの素直な減速プロファイルを保ちつつ、目標間際の
        // オーバーシュート抑制効果だけを残す狙い。
        const bool turn_fine_phase = (fabsf(err) <= TURN_FINE_ENABLE_RAD);
        if (turn_fine_phase) {
            turn_integ += err * dt_s;
        }
        // アンチワインドアップ: 積分項単独で最大速度を超えないようクランプ
        if (TURN_KI_MPS_PER_RAD_S > 0.0f) {
            const float integ_max = TURN_MAX_SPEED_MPS / TURN_KI_MPS_PER_RAD_S;
            if (turn_integ > integ_max) turn_integ = integ_max;
            if (turn_integ < -integ_max) turn_integ = -integ_max;
        }

        const float gyro_z = sensors.get_gyro_z();
        const float d_term = turn_fine_phase ? (-TURN_KD_MPS_PER_RADPS * gyro_z) : 0.0f;
        const float pid_out = TURN_KP_MPS_PER_RAD * err
                             + TURN_KI_MPS_PER_RAD_S * turn_integ
                             + d_term;

        float v_target = fabsf(pid_out);
        if (v_target > TURN_MAX_SPEED_MPS) v_target = TURN_MAX_SPEED_MPS;
        if (v_target < TURN_MIN_SPEED_MPS) v_target = TURN_MIN_SPEED_MPS;

        // 加減速をなめらかにする（slew rate limit）
        const float dv_max = TURN_ACCEL_MPS2 * dt_s;
        turn_speed_cmd_mps = slew_rate_limit(turn_speed_cmd_mps, v_target, dv_max);

        const float target_rel = err; // 現在から見た残り角度
        motion.turn_in_place(turn_speed_cmd_mps, target_rel);

        // デバッグ用
        target_vr_mps = (err >= 0.0f) ? +turn_speed_cmd_mps : -turn_speed_cmd_mps;
        target_vl_mps = (err >= 0.0f) ? -turn_speed_cmd_mps : +turn_speed_cmd_mps;

        {
            static int dbg_count = 0;
            dbg_count++;
            if (dbg_count >= 50) {  // 20Hz
                dbg_count = 0;
                // #V,cmd,vr,vl,ur,ul,gz,ang,rem (remはTURNのみ残り角[rad])
                char msg[64];
                snprintf(msg, sizeof(msg),
                         "#V,%.0f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.3f\n",
                         turn_speed_cmd_mps * 1000.0f,
                         motion.get_vr_filt_mps() * 1000.0f,
                         motion.get_vl_filt_mps() * 1000.0f,
                         motion.get_duty_r(), motion.get_duty_l(),
                         gyro_z, now_ang, err);
                enqueue_msg_line(msg);
            }
        }
    }
}

bool updateQstp(float dt_s) {
    if (!qstp_active) return false;

    // 現在速度が十分低ければ停止完了
    if (qstp_v_cmd_mmps <= 5.0f) {
        qstp_active = false;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
        
        // 目標距離との差分を計算してQSTPDONEコマンドで送り返す
        float remaining_dist = 0.0f;
        if (qstp_original_goal_dist_mm > 0.0f) {
            const float current_dist = sensors.get_distance();
            remaining_dist = qstp_original_goal_dist_mm - current_dist;
        }
        
        char msg[64];
        snprintf(msg, sizeof(msg), "QSTPDONE,%.1f\n", remaining_dist);
        enqueue_msg_line(msg);
        
        return false;
    }

    // 最大減速度で減速
    const float decel_delta = QSTP_DECEL_MMPS2 * dt_s;
    qstp_v_cmd_mmps -= decel_delta;
    if (qstp_v_cmd_mmps < 0.0f) {
        qstp_v_cmd_mmps = 0.0f;
    }

    const float v_cmd_mps = qstp_v_cmd_mmps / 1000.0f;

    // 角度フィードバック: 現在角度と目標角度の差分
    const float angle_error = sensors.get_angle() - qstp_target_angle_rad;
    const float angle_correction = ANGLE_FB_GAIN * angle_error;

    // 角速度フィードバック: 角速度をゼロに保つ
    const float gyro_z = sensors.get_gyro_z();
    const float rate_correction = ANGULAR_RATE_FB_GAIN * gyro_z;
    
    // 壁センサフィードバック
    const float wall_correction = calculate_wall_correction(sensors);

    // 最終速度計算: 車輪の速度差で補正
    const float vr = v_cmd_mps - angle_correction - rate_correction + wall_correction;
    const float vl = v_cmd_mps + angle_correction + rate_correction - wall_correction;

    target_vr_mps = vr;
    target_vl_mps = vl;
    // 上で計算したvr/vlの差分をそのままlateral_correctionとして渡す
    // （motion.forward()内部で speed±corr に展開されるため、平均速度＋補正量で渡す）
    const float lateral_correction = angle_correction + rate_correction - wall_correction;

    {
        static int dbg_count = 0;
        dbg_count++;
        if (dbg_count >= 50) {  // 20Hz
            dbg_count = 0;
            const float remain_mm = (qstp_original_goal_dist_mm > 0.0f)
                ? (qstp_original_goal_dist_mm - sensors.get_distance())
                : 0.0f;
            // #V,cmd,vr,vl,ur,ul,gz,ang,rem
            char msg[64];
            snprintf(msg, sizeof(msg),
                     "#V,%.0f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.1f\n",
                     qstp_v_cmd_mmps,
                     motion.get_vr_filt_mps() * 1000.0f,
                     motion.get_vl_filt_mps() * 1000.0f,
                     motion.get_duty_r(), motion.get_duty_l(),
                     gyro_z, sensors.get_angle(), remain_mm);
            enqueue_msg_line(msg);
        }
    }

    motion.forward(v_cmd_mps, lateral_correction);

    return true;
}

// ========================================
// Core0 リアルタイムタスク
// ========================================

void Core0RealtimeTask(void* parameter) {
    // ハイスピードタイマー(1ms)の初期化
    high_speed_timer = timerBegin(1000); // 1000Hz = 1ms period
    timerAttachInterrupt(high_speed_timer, &onHighSpeedTimer);
    timerAlarm(high_speed_timer, 1, true, 0); // 1ms interval, auto-reload

    uint32_t last_tick = 0;
    uint32_t time_delta = 0;

    while(1){
        // 1msごとの待機
        time_delta = waitTick(last_tick);

        // センサーデータの更新
        sensors.update(time_delta);

        // コマンドキューの処理（loop() -> Core0）
        processCommandQueue();

        const float dt_s = static_cast<float>(time_delta) / 1000.0f;

        // 各モーション状態の更新
        if (!updateLatch(dt_s) && !updateJog(dt_s)) {
            if (!updateQstp(dt_s)) {
                updateForward(dt_s);
                updateStop(dt_s);
                updateTurn(dt_s);
            }
        }

        // ジャイロキャリブレーション完了チェック
        if (gyro_calib_done_pending && !sensors.is_calibrating()) {
            gyro_calib_done_pending = false;
            char msg[64];
            float offset = sensors.get_gyro_offset();
            snprintf(msg, sizeof(msg), "#Gyro offset=%.6f rad/s\n", offset);
            enqueue_msg_line(msg);
            enqueue_msg_line("DONE\n");
        }

        // Motion controller update (speed PID)
        motion.update(time_delta);
    }
}

void setup() {
    Serial.begin(3000000);
    delay(100);
    
    // 1. ハードウェアWDT無効化
    disableCore0WDT();
    
    // WiFi/Bluetooth無効化
    WiFi.mode(WIFI_OFF);
    btStop();

    // 共有SPIとペリフェラルの初期化
    init_imu_spi();
    motor.begin();
    encoder.begin();
    imu.begin();
    wall_sensor.begin();
    battery.begin();
    led.begin();
    fan.begin();
    servo.begin();
    ball_sensor.begin();

    // 起動時に赤色LEDを消灯
    led.red_off();

    // コマンドキューの作成
    cmd_queue = xQueueCreate(16, sizeof(Command));
    if (cmd_queue == NULL) {
        Serial.printf("#Failed to create command queue!\n");
        while(1);
    }

    // Core0 -> Core1 メッセージキューの作成
    msg_queue = xQueueCreate(128, sizeof(MsgLine));
    if (msg_queue == NULL) {
        Serial.printf("#Failed to create message queue!\n");
        while(1);
    }

    // Core0でハイスピードタスクを開始
    BaseType_t result = xTaskCreatePinnedToCore(
        Core0RealtimeTask,
        "Core0RT",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // 最高優先度
        NULL,
        0  // Core 0
    );
    
    if (result != pdPASS) {
        Serial.println("#Failed to create Core0 task!\n");
        while(1);
    }
    
    Serial.printf("#Core 0 realtime task started\n");
    Serial.printf("#System ready\n");
}
 
// Core1のループ関数
void loop() {
    // Core0からのメッセージ出力（Serial.printfはCore1側でのみ行う）
    {
        MsgLine m;
        while (msg_queue && xQueueReceive(msg_queue, &m, 0) == pdTRUE) {
            Serial.print(m.text);
        }
    }

    // センサーデータ出力は周期送信を止め、PCからの要求に応答して送る方式に変更した
    
    // UARTコマンド受信例
    if (Serial.available()) {
        // e.g. "MOT,100,100\n"
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("MOT")) {
            // モーター速度設定: MOT,100,100
            // 期待フォーマット: "MOT,<right_speed>,<left_speed>"
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1) {
                Command q;
                q.cmd_id = CMD_SET_MOTOR_SPEED;
                q.parameter.set_motor_speed.right_speed = cmd.substring(comma1 + 1, comma2).toInt();
                q.parameter.set_motor_speed.left_speed = cmd.substring(comma2 + 1).toInt();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#Motor: R=%d, L=%d\n",
                                  q.parameter.set_motor_speed.right_speed,
                                  q.parameter.set_motor_speed.left_speed);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid MOT format\n");
            }
        } else if (cmd.startsWith("WALL")) {
            // 壁センサLEDの有効/無効: WALL,1 または WALL,0
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                int en = cmd.substring(comma1 + 1).toInt();
                bool enabled = (en != 0);
                wall_sensor.set_enabled(enabled);
                Serial.printf("#WallSensor enabled=%d\n", enabled ? 1 : 0);
            } else {
                Serial.printf("#Invalid WALL format\n");
            }
        } else if (cmd.startsWith("FAN,")) {
            // 吸引ファンPWM速度指定: FAN,<speed>（0-255、0=停止）
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                int speed = cmd.substring(comma1 + 1).toInt();
                if (speed < 0) speed = 0;
                if (speed > 255) speed = 255;
                fan.set_speed(static_cast<uint8_t>(speed));
                Serial.printf("#FAN speed=%d\n", speed);
            } else {
                Serial.printf("#Invalid FAN format\n");
            }
        } else if (cmd.startsWith("SRV,")) {
            // RCサーボ角度設定: SRV,<angle>（0-180度）
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                int angle = cmd.substring(comma1 + 1).toInt();
                if (angle < 0) angle = 0;
                if (angle > 180) angle = 180;
                servo.set_angle(static_cast<uint8_t>(angle));
                Serial.printf("#SRV angle=%d\n", angle);
            } else {
                Serial.printf("#Invalid SRV format\n");
            }
        } else if (cmd == "SRVOFF") {
            // RCサーボのトルクオフ（脱力）
            servo.detach();
            Serial.printf("#SRVOFF\n");
        } else if (cmd.startsWith("BALL,")) {
            // ボールセンサしきい値設定: BALL,<threshold>（0-4095）
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                int threshold = cmd.substring(comma1 + 1).toInt();
                if (threshold < 0) threshold = 0;
                if (threshold > 4095) threshold = 4095;
                ball_sensor.set_threshold(static_cast<uint16_t>(threshold));
                Serial.printf("#BALL threshold=%d\n", threshold);
            } else {
                Serial.printf("#Invalid BALL format\n");
            }
        } else if (cmd.startsWith("FWD,")) {
            // FWD: FWD,<speed_mmps>,<accel_mmps2>,<distance_mm>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            int comma3 = (comma2 >= 0) ? cmd.indexOf(',', comma2 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
                Command q;
                q.cmd_id = CMD_FORWARD;
                q.parameter.forward.speed_mmps = cmd.substring(comma1 + 1, comma2).toFloat();
                q.parameter.forward.accel_mmps2 = cmd.substring(comma2 + 1, comma3).toFloat();
                q.parameter.forward.distance_mm = cmd.substring(comma3 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#FWD speed=%.1fmm/s accel=%.1fmm/s^2 dist=%.1fmm\n",
                                  q.parameter.forward.speed_mmps,
                                  q.parameter.forward.accel_mmps2,
                                  q.parameter.forward.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid FWD format\n");
            }
        } else if (cmd.startsWith("STOP,")) {
            // STOP: STOP,<speed_mmps>,<accel_mmps2>,<distance_mm>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            int comma3 = (comma2 >= 0) ? cmd.indexOf(',', comma2 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
                Command q;
                q.cmd_id = CMD_STOP;
                q.parameter.stop.speed_mmps = cmd.substring(comma1 + 1, comma2).toFloat();
                q.parameter.stop.accel_mmps2 = cmd.substring(comma2 + 1, comma3).toFloat();
                q.parameter.stop.distance_mm = cmd.substring(comma3 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#STOP speed=%.1fmm/s accel=%.1fmm/s^2 dist=%.1fmm\n",
                                  q.parameter.stop.speed_mmps,
                                  q.parameter.stop.accel_mmps2,
                                  q.parameter.stop.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid STOP format\n");
            }
        } else if (cmd.startsWith("TURN,")) {
            // TURN: TURN,<angle_rad>
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                Command q;
                q.cmd_id = CMD_TURN;
                q.parameter.turn_target_rad = cmd.substring(comma1 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#TURN angle=%.4frad\n", q.parameter.turn_target_rad);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid TURN format\n");
            }
        } else if (cmd == "LFWD") {
            // LFWD: 10mm/sでLSTOPまで前進
            Command q;
            q.cmd_id = CMD_LATCH_START;
            q.parameter.latch.mode = static_cast<uint8_t>(LatchMode::FWD);
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#LFWD\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "LBACK") {
            // LBACK: 10mm/sでLSTOPまで後退
            Command q;
            q.cmd_id = CMD_LATCH_START;
            q.parameter.latch.mode = static_cast<uint8_t>(LatchMode::BACK);
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#LBACK\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "LTURNL") {
            // LTURNL: 90deg/sでLSTOPまで左旋回
            Command q;
            q.cmd_id = CMD_LATCH_START;
            q.parameter.latch.mode = static_cast<uint8_t>(LatchMode::TURN_L);
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#LTURNL\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "LTURNR") {
            // LTURNR: 90deg/sでLSTOPまで右旋回
            Command q;
            q.cmd_id = CMD_LATCH_START;
            q.parameter.latch.mode = static_cast<uint8_t>(LatchMode::TURN_R);
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#LTURNR\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "LSTOP") {
            // LSTOP: L*動作停止
            Command q;
            q.cmd_id = CMD_LATCH_STOP;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#LSTOP\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd.startsWith("JOGFWD,")) {
            // JOGFWD: 低速で指定距離前進 JOGFWD,<distance_mm>
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                Command q;
                q.cmd_id = CMD_JOG_START;
                q.parameter.jog.is_forward = true;
                q.parameter.jog.distance_mm = cmd.substring(comma1 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#JOGFWD dist=%.1fmm\n", q.parameter.jog.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid JOGFWD format\n");
            }
        } else if (cmd.startsWith("JOGBACK,")) {
            // JOGBACK: 低速で指定距離後退 JOGBACK,<distance_mm>
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                Command q;
                q.cmd_id = CMD_JOG_START;
                q.parameter.jog.is_forward = false;
                q.parameter.jog.distance_mm = cmd.substring(comma1 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#JOGBACK dist=%.1fmm\n", q.parameter.jog.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid JOGBACK format\n");
            }
        } else if (cmd == "RDST") {
            // 距離リセット（オドメトリ）
            Command q;
            q.cmd_id = CMD_RESET_DISTANCE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#RDST\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "RANG") {
            // 角度リセット（オドメトリ）
            Command q;
            q.cmd_id = CMD_RESET_ANGLE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#RANG\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "GCAL") {
            // ジャイロキャリブレーション
            Command q;
            q.cmd_id = CMD_GYRO_CALIBRATE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#GCAL\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "QSTP") {
            // クイック停止
            Command q;
            q.cmd_id = CMD_QSTP;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#QSTP\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "SEN") {
            // PCからの要求に応答して1行だけSENを送る
            float gyro = sensors.get_gyro_z();      // rad/s
            float vbatt = sensors.get_battery_voltage();
            uint16_t lf = sensors.get_lf();
            uint16_t ls = sensors.get_ls();
            uint16_t rs = sensors.get_rs();
            uint16_t rf = sensors.get_rf();
            uint16_t enc_r = sensors.get_right_wheel_angle();
            uint16_t enc_l = sensors.get_left_wheel_angle();
            float odo_dist = sensors.get_distance();
            float odo_ang = sensors.get_angle();    // rad
            uint16_t ball_raw = ball_sensor.read_raw();
            bool ball_det = ball_sensor.detect();
            Serial.printf("SEN,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%u,%u\n",
                          gyro, vbatt, lf, ls, rs, rf, enc_r, enc_l, odo_dist, odo_ang,
                          ball_raw, ball_det ? 1 : 0);
        } else {
            // デバッグ用: 不明コマンド
            Serial.printf("#Unknown cmd: %s\n", cmd.c_str());
        }
    }
    
    delay(1);
}
