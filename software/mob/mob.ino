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
#include "place_controller.h"
#include "path_controller.h"
#include "params.h"
#include "fan.h"
#include "servo.h"
#include "ball_sensor.h"
#include <math.h>

// Target wheel speed [m/s] (updated from MOT command via cmd_queue)
static float target_vr_mps = 0.0f;
static float target_vl_mps = 0.0f;

// Core0が今どのモーションを実行中か。個別boolの組み合わせだと
// 「他の全フラグをfalseにする」処理を新しいモーション追加のたびに
// 手で書く必要があり書き漏れが起きやすいため、単一のenumにして
// 新しい状態への代入1回だけで「他の状態ではない」が構造的に保証される
// ようにしている(2026-07-24導入、2026-08-02にFWD/STOP/TURN等の旧移動系
// 状態を削除しPLACE_HOLDのみに整理)。
enum class MotionState : uint8_t {
    IDLE,
    PLACE_HOLD,   // その場静止制御(place_controller.cpp、2026-08-02〜)
    PATH_FOLLOW,  // 仮想ターゲット追従パス走行(path_controller.cpp、2026-08-02〜)
};
static MotionState motion_state = MotionState::IDLE;

// ジャイロキャリブレーション状態
static bool gyro_calib_done_pending = false;  // キャリブレーション完了時にDONEを返す

// グローバルなクラスは、上から順に初期化される
Led led;
WallSensor wall_sensor;
IMU imu(get_imu_spi());
Motor motor;
Encoder encoder;
Battery battery;
Sensors sensors(imu, wall_sensor, battery, encoder);
MotionController motion(motor, sensors);
PlaceController place_controller(motor, sensors);
PathController path_controller(motor, sensors);
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

struct SetDutyCommand {
    int16_t right_duty; // 生duty（-1023〜+1023、速度PID非経由）
    int16_t left_duty;  // 生duty（-1023〜+1023、速度PID非経由）
};

// PATTERN走行パス(PC側からPADDで1区間ずつ送って組み立てる、2026-08-02〜)
struct AddStraightCommand {
    float distance_mm;
    float v_start_mmps;
    float v_cruise_mmps;
    float v_end_mmps;
};

struct AddSlalomCommand {
    float v_mmps;
    float dir;        // +1=左(CCW), -1=右(CW)
    float radius_mm;
    float angle_rad;  // ASCIIコマンドは度指定、パース時にradへ変換する
};

enum CommandID : uint8_t {
    CMD_SET_MOTOR_SPEED = 0x01,
    CMD_RESET_DISTANCE = 0x06,
    CMD_RESET_ANGLE = 0x07,
    CMD_GYRO_CALIBRATE = 0x09,
    CMD_SET_DUTY_DIRECT = 0x0E,
    CMD_SET_ANGLE = 0x0F,
    CMD_PLACE_HOLD_START = 0x11,
    CMD_TURN = 0x12,
    CMD_PATTERN_CLEAR = 0x14,
    CMD_PATTERN_ADD_STRAIGHT = 0x15,
    CMD_PATTERN_ADD_SLALOM = 0x16,
    CMD_PATTERN_RUN = 0x17,
};

// PATTERN走行パスの格納先。PCLEAR/PADDで組み立て、PRUNで
// path_controller.start()へ渡す(2026-08-02、当初mob.ino内に固定
// ハードコードしていたが、PC側から任意のパスを送れるように変更)。
// PathController::start()はポインタを保持するだけでコピーしないため、
// この配列はプログラム全体の寿命を通じて有効な静的領域である必要がある。
static constexpr size_t MAX_PATTERN_SEGMENTS = 32;
static PathController::Segment g_pattern_segments[MAX_PATTERN_SEGMENTS];
static size_t g_pattern_segment_count = 0;

struct Command {
    CommandID cmd_id;
    union {
        SetMotorSpeedCommand set_motor_speed;
        SetDutyCommand set_duty;
        float set_angle_rad;
        AddStraightCommand add_straight;
        AddSlalomCommand add_slalom;
        float turn_angle_rad;
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
void handleSetDutyCommand(const SetDutyCommand& cmd);
void processCommandQueue();
void updatePlaceHold(float dt_s);
void updatePathFollow(float dt_s);
int split_csv(const String& s, String* out, int max_tokens);

// カンマ区切り文字列をトークン配列に分割する(PADD,STRAIGHT/SLALOMの
// 可変引数パース用)。戻り値は分割できたトークン数(max_tokensで打ち切り、
// 末尾トークンは残り全体)。
int split_csv(const String& s, String* out, int max_tokens) {
    int count = 0;
    int start = 0;
    while (count < max_tokens) {
        int comma = s.indexOf(',', start);
        if (comma < 0) {
            out[count++] = s.substring(start);
            break;
        }
        out[count++] = s.substring(start, comma);
        start = comma + 1;
    }
    return count;
}

// ========================================
// コマンド処理関数の実装
// ========================================

void handleSetMotorSpeedCommand(const SetMotorSpeedCommand& cmd) {
    const float vr = static_cast<float>(cmd.right_speed) / 1000.0f;
    const float vl = static_cast<float>(cmd.left_speed) / 1000.0f;

    // 手動MOTが来たらほかのモーションは停止（競合回避）
    motion_state = MotionState::IDLE;

    target_vr_mps = vr;
    target_vl_mps = vl;
    motion.forward((vr + vl) * 0.5f);
}

void handleSetDutyCommand(const SetDutyCommand& cmd) {
    // 手動DUTYが来たらほかのモーションは停止（競合回避、MOTと同様）
    motion_state = MotionState::IDLE;

    motion.set_duty_direct(cmd.right_duty, cmd.left_duty);
}

void processCommandQueue() {
    Command q;
    while (xQueueReceive(cmd_queue, &q, 0) == pdTRUE) {
        switch (q.cmd_id) {
            case CMD_SET_MOTOR_SPEED:
                handleSetMotorSpeedCommand(q.parameter.set_motor_speed);
                break;

            case CMD_RESET_DISTANCE:
                sensors.reset_distance();
                enqueue_msg_line("#distance reset\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_RESET_ANGLE:
                sensors.reset_angle();
                enqueue_msg_line("#angle reset\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_SET_ANGLE:
                // 外部の絶対基準(カメラ等)で角度を上書きする。
                // 走行中の制御ループが参照する目標角には触れないため、
                // 呼ぶのは停止中に限ること。
                sensors.set_angle(q.parameter.set_angle_rad);
                enqueue_msg_line("#angle set\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_GYRO_CALIBRATE:
                // ジャイロキャリブレーション開始（非ブロッキング）
                enqueue_msg_line("#Gyro calibration start...\n");
                sensors.calibrate_gyro();
                gyro_calib_done_pending = true;
                break;

            case CMD_SET_DUTY_DIRECT:
                handleSetDutyCommand(q.parameter.set_duty);
                break;

            case CMD_PLACE_HOLD_START:
                // 競合回避: ほかのモーションを停止
                motion_state = MotionState::PLACE_HOLD;
                place_controller.start();
                enqueue_msg_line("#PLACE_HOLD: start\n");
                break;

            case CMD_TURN:
                // 競合回避: ほかのモーションを停止
                motion_state = MotionState::PLACE_HOLD;
                place_controller.start_turn(q.parameter.turn_angle_rad);
                enqueue_msg_line("#PLACE_HOLD: turn start\n");
                break;

            case CMD_PATTERN_CLEAR:
                g_pattern_segment_count = 0;
                enqueue_msg_line("#PATTERN: cleared\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_PATTERN_ADD_STRAIGHT:
                if (g_pattern_segment_count < MAX_PATTERN_SEGMENTS) {
                    PathController::Segment& seg = g_pattern_segments[g_pattern_segment_count++];
                    seg.type = PathController::SegmentType::STRAIGHT;
                    seg.distance_mm = q.parameter.add_straight.distance_mm;
                    seg.v_start_mmps = q.parameter.add_straight.v_start_mmps;
                    seg.v_cruise_mmps = q.parameter.add_straight.v_cruise_mmps;
                    seg.v_end_mmps = q.parameter.add_straight.v_end_mmps;
                    seg.v_mmps = 0.0f;
                    seg.dir = 0.0f;
                    seg.radius_mm = 0.0f;
                    seg.angle_rad = 0.0f;
                    enqueue_msg_line("#PATTERN: add straight\n");
                    enqueue_msg_line("DONE\n");
                } else {
                    enqueue_msg_line("#PATTERN: buffer full\n");
                    enqueue_msg_line("ERR\n");
                }
                break;

            case CMD_PATTERN_ADD_SLALOM:
                if (g_pattern_segment_count < MAX_PATTERN_SEGMENTS) {
                    PathController::Segment& seg = g_pattern_segments[g_pattern_segment_count++];
                    seg.type = PathController::SegmentType::SLALOM;
                    seg.distance_mm = 0.0f;
                    seg.v_start_mmps = 0.0f;
                    seg.v_cruise_mmps = 0.0f;
                    seg.v_end_mmps = 0.0f;
                    seg.v_mmps = q.parameter.add_slalom.v_mmps;
                    seg.dir = q.parameter.add_slalom.dir;
                    seg.radius_mm = q.parameter.add_slalom.radius_mm;
                    seg.angle_rad = q.parameter.add_slalom.angle_rad;
                    enqueue_msg_line("#PATTERN: add slalom\n");
                    enqueue_msg_line("DONE\n");
                } else {
                    enqueue_msg_line("#PATTERN: buffer full\n");
                    enqueue_msg_line("ERR\n");
                }
                break;

            case CMD_PATTERN_RUN:
                // 競合回避: ほかのモーションを停止
                motion_state = MotionState::PATH_FOLLOW;
                path_controller.start(g_pattern_segments, g_pattern_segment_count);
                enqueue_msg_line("#PATH_FOLLOW: pattern start\n");
                break;

            default:
                break;
        }
    }
}

void updatePlaceHold(float dt_s) {
    if (motion_state != MotionState::PLACE_HOLD) return;
    place_controller.update(dt_s);

    static int dbg_count = 0;
    dbg_count++;
    if (dbg_count >= 50) {  // 20Hz
        dbg_count = 0;
        // #P,vr,vl,vsum,duty
        char msg[64];
        snprintf(msg, sizeof(msg), "#P,%.1f,%.1f,%.1f,%d\n",
                 place_controller.get_vr_mps() * 1000.0f,
                 place_controller.get_vl_mps() * 1000.0f,
                 place_controller.get_v_sum_mps() * 1000.0f,
                 place_controller.get_duty());
        enqueue_msg_line(msg);

        if (place_controller.is_turning()) {
            // #A,target_angle,actual_angle,omega_target,duty_diff
            char msg2[64];
            snprintf(msg2, sizeof(msg2), "#A,%.4f,%.4f,%.2f,%d\n",
                     place_controller.get_target_angle_rad(),
                     sensors.get_angle(),
                     place_controller.get_omega_target_radps(),
                     place_controller.get_duty_diff());
            enqueue_msg_line(msg2);
        }
    }
}

void updatePathFollow(float dt_s) {
    if (motion_state != MotionState::PATH_FOLLOW) return;
    path_controller.update(dt_s);

    static int dbg_count = 0;
    dbg_count++;
    if (dbg_count >= 50) {  // 20Hz
        dbg_count = 0;
        // #T,seg,tx,ty,rx,ry,rtheta,dist,hdg_err
        char msg[64];
        snprintf(msg, sizeof(msg), "#T,%u,%.0f,%.0f,%.0f,%.0f,%.3f,%.1f,%.3f\n",
                 static_cast<unsigned>(path_controller.get_seg_index()),
                 path_controller.get_target_x_mm(),
                 path_controller.get_target_y_mm(),
                 path_controller.get_robot_x_mm(),
                 path_controller.get_robot_y_mm(),
                 path_controller.get_robot_theta_rad(),
                 path_controller.get_dist_to_target_mm(),
                 path_controller.get_heading_error_rad());
        enqueue_msg_line(msg);
    }
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

        // motion_stateが単一の排他状態のため、update*は自分の状態で
        // なければ何もせず即returnする。
        updatePlaceHold(dt_s);
        updatePathFollow(dt_s);

        // ジャイロキャリブレーション完了チェック
        if (gyro_calib_done_pending && !sensors.is_calibrating()) {
            gyro_calib_done_pending = false;
            char msg[64];
            float offset = sensors.get_gyro_offset();
            snprintf(msg, sizeof(msg), "#Gyro offset=%.6f rad/s\n", offset);
            enqueue_msg_line(msg);
            enqueue_msg_line("DONE\n");
        }

        // Motion controller update (speed PID)。PLACE_HOLD/PATH_FOLLOW中は
        // place_controller/path_controllerが直接モーターを駆動するため、
        // motion(旧MotionController)側の速度PIDが競合して上書きしないよう
        // ここで止める(motion側の目標vr_ref/vl_refはその間一切更新
        // されないため、素通しすると古い目標値に基づいてduty=0等を
        // 毎tick書き込んでしまう)。
        if (motion_state != MotionState::PLACE_HOLD && motion_state != MotionState::PATH_FOLLOW) {
            motion.update(time_delta);
        }
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

    // 機体固有パラメータ: NVSに保存済みならそれを読み込む(無ければ
    // params.cppのビルド時デフォルトのまま)
    params_begin();

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
        } else if (cmd.startsWith("DUTY,")) {
            // 生duty直接指令（校正・診断用、速度PID非経由）: DUTY,<right_duty>,<left_duty>
            // 範囲: -1023〜+1023
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1) {
                Command q;
                q.cmd_id = CMD_SET_DUTY_DIRECT;
                q.parameter.set_duty.right_duty = cmd.substring(comma1 + 1, comma2).toInt();
                q.parameter.set_duty.left_duty = cmd.substring(comma2 + 1).toInt();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#Duty: R=%d, L=%d\n",
                                  q.parameter.set_duty.right_duty,
                                  q.parameter.set_duty.left_duty);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid DUTY format\n");
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
        } else if (cmd == "PGET") {
            // 全パラメータを列挙: PVAL,<name>,<value> を1行ずつ、
            // 最後に PLISTEND
            for (size_t i = 0; i < PARAM_COUNT; i++) {
                float value = 0.0f;
                params_get(PARAM_TABLE[i].name, value);
                Serial.printf("PVAL,%s,%.6f\n", PARAM_TABLE[i].name, value);
            }
            Serial.printf("PLISTEND\n");
        } else if (cmd.startsWith("PGET,")) {
            // 単一パラメータ取得: PGET,<name>
            String name = cmd.substring(5);
            float value = 0.0f;
            if (params_get(name.c_str(), value)) {
                Serial.printf("PVAL,%s,%.6f\n", name.c_str(), value);
            } else {
                Serial.printf("#Unknown param: %s\n", name.c_str());
            }
        } else if (cmd.startsWith("PSET,")) {
            // パラメータ即時変更(RAM上のみ): PSET,<name>,<value>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            if (comma2 > 0) {
                String name = cmd.substring(comma1 + 1, comma2);
                float value = cmd.substring(comma2 + 1).toFloat();
                if (params_set(name.c_str(), value)) {
                    Serial.printf("#PSET %s=%.6f\n", name.c_str(), value);
                } else {
                    Serial.printf("#Unknown param: %s\n", name.c_str());
                }
            } else {
                Serial.printf("#Invalid PSET format\n");
            }
        } else if (cmd == "PSAVE") {
            // 現在のパラメータを丸ごとNVSへ保存(機体固有の恒久設定)
            if (params_save()) {
                Serial.printf("DONE\n");
            } else {
                Serial.printf("#PSAVE failed\n");
            }
        } else if (cmd == "PLOAD") {
            // NVSから読み込みRAMへ反映(未保存の項目は現在値を維持)
            if (!params_load()) {
                Serial.printf("#PLOAD: no saved params\n");
            }
            Serial.printf("DONE\n");
        } else if (cmd == "PRESET") {
            // RAM上のパラメータをビルド時デフォルトへ戻す(NVSは変更しない)
            params_reset_to_defaults();
            Serial.printf("DONE\n");
        } else if (cmd == "HOLD") {
            // HOLD: その場静止制御を開始(左右輪速度の和をゼロへ、place_controller.cpp)
            Command q;
            q.cmd_id = CMD_PLACE_HOLD_START;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#HOLD\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd.startsWith("TURN,")) {
            // TURN: その場旋回(角度制御、台形速度プロファイル): TURN,<angle_rad>(正=左/CCW)
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                Command q;
                q.cmd_id = CMD_TURN;
                q.parameter.turn_angle_rad = cmd.substring(comma1 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#TURN angle=%.4frad\n", q.parameter.turn_angle_rad);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid TURN format\n");
            }
        } else if (cmd == "PCLEAR") {
            // PCLEAR: PATTERN走行パスのバッファをクリア(PADDの前に毎回呼ぶ)
            Command q;
            q.cmd_id = CMD_PATTERN_CLEAR;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#PCLEAR\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd.startsWith("PADD,STRAIGHT,")) {
            // PADD,STRAIGHT,<distance_mm>,<v_start_mmps>,<v_cruise_mmps>,<v_end_mmps>
            String tok[6];
            int n = split_csv(cmd, tok, 6);
            if (n == 6) {
                Command q;
                q.cmd_id = CMD_PATTERN_ADD_STRAIGHT;
                q.parameter.add_straight.distance_mm = tok[2].toFloat();
                q.parameter.add_straight.v_start_mmps = tok[3].toFloat();
                q.parameter.add_straight.v_cruise_mmps = tok[4].toFloat();
                q.parameter.add_straight.v_end_mmps = tok[5].toFloat();
                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#PADD STRAIGHT\n");
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid PADD,STRAIGHT format\n");
            }
        } else if (cmd.startsWith("PADD,SLALOM,")) {
            // PADD,SLALOM,<v_mmps>,<L|R>,<radius_mm>,<angle_deg>(度指定、桁数節約のためradではなくdeg)
            String tok[6];
            int n = split_csv(cmd, tok, 6);
            if (n == 6 && (tok[3] == "L" || tok[3] == "R")) {
                Command q;
                q.cmd_id = CMD_PATTERN_ADD_SLALOM;
                q.parameter.add_slalom.v_mmps = tok[2].toFloat();
                q.parameter.add_slalom.dir = (tok[3] == "L") ? 1.0f : -1.0f;
                q.parameter.add_slalom.radius_mm = tok[4].toFloat();
                q.parameter.add_slalom.angle_rad = tok[5].toFloat() * DEG_TO_RAD;
                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#PADD SLALOM\n");
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid PADD,SLALOM format\n");
            }
        } else if (cmd == "PRUN") {
            // PRUN: PCLEAR/PADDで組み立てたPATTERN走行パスを開始
            Command q;
            q.cmd_id = CMD_PATTERN_RUN;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#PRUN\n");
            } else {
                Serial.printf("#Queue full!\n");
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
        } else if (cmd.startsWith("SANG,")) {
            // 角度上書き（外部の絶対基準での補正用）: SANG,<angle_rad>
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                Command q;
                q.cmd_id = CMD_SET_ANGLE;
                q.parameter.set_angle_rad = cmd.substring(comma1 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#SANG angle=%.4frad\n", q.parameter.set_angle_rad);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid SANG format\n");
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
