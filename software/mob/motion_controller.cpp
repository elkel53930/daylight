#include "motion_controller.h"
#include <math.h>

// Tuning constants
// 実測プラントゲイン ≈ 3mm/s per duty (2026-07-19 テレメトリ: duty96↔300mm/s)。
// KP=800はループゲイン≈2.4で振動していたため、FF主体+P縮小(≈0.9)に変更。
static constexpr float DEFAULT_KP = 300.0f;
// KI=60 は弱すぎて実質P制御になり、負荷・電圧変動がそのまま速度変動に
// 現れていた(2026-07-19)。定常dutyを積分項が数百msで担える値に引き上げ。
static constexpr float DEFAULT_KI = 3000.0f;
static constexpr float DEFAULT_KD = 0.0f;

// 速度フィードフォワード: 目標速度[m/s]→duty。
// VBATT_NOM 時の値。実効ゲインは電圧に反比例するため vbatt でスケーリングする
// (2026-07-19: 計測毎に duty↔速度 の関係が±10%以上ずれる原因が電圧変動だった)。
// 定常テレメトリ実測 duty100 ↔ 約370mm/s(vbatt≈7.5V)から逆算。
static constexpr float KF_DUTY_PER_MPS = 250.0f;
static constexpr float VBATT_NOM = 8.0f;  // KF を校正した基準電圧 [V]

// 速度測定(1ms毎のエンコーダ差分)の量子化ノイズを抑えるLPF係数
// 0.3では±100mm/s級のスパイク(ジャイロと矛盾=計測ノイズ)が残った(2026-07-19)
static constexpr float SPEED_LPF_ALPHA = 0.12f;

float MotionController::SpeedPID::step(float err, float dt_s) {
    if (dt_s <= 0) return 0.0f;
    integ += err * dt_s;
    // アンチワインドアップ: 積分項単独で出力範囲を超えないようクランプ
    if (ki > 0.0f) {
        const float integ_max = out_max / ki;
        const float integ_min = out_min / ki;
        if (integ > integ_max) integ = integ_max;
        if (integ < integ_min) integ = integ_min;
    }
    float deriv = (err - prev_err) / dt_s;
    prev_err = err;
    float u = kp * err + ki * integ + kd * deriv;
    if (u > out_max) u = out_max;
    if (u < out_min) u = out_min;
    return u;
}

void MotionController::SpeedPID::reset() {
    integ = 0.0f;
    prev_err = 0.0f;
}

MotionController::MotionController(Motor& motor, Sensors& sensors)
    : motor_(motor), sensors_(sensors) {
    // out_min/out_maxは「D の duty」範囲ではなく「speed」範囲で指定
    // T: −255〜+255 → D: −1023〜+1023
    pid_r_ = {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0.0f, 0.0f, -1023.0f, 1023.0f};
    pid_l_ = {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0.0f, 0.0f, -1023.0f, 1023.0f};
}

void MotionController::update(uint32_t dt_ms) {
    apply_speed_pid(dt_ms);
}

void MotionController::forward(float speed_mps, float lateral_error) {
    mode_ = Mode::FORWARD;

    const float corr = k_lateral_ * lateral_error;
    set_targets_mps(speed_mps - corr, speed_mps + corr);
}

void MotionController::backward(float speed_mps) {
    mode_ = Mode::BACKWARD;
    set_targets_mps(-fabsf(speed_mps), -fabsf(speed_mps));
}

void MotionController::turn_in_place(float speed_mps, float target_angle_rad) {
    // 完了判定は呼び出し側（mob.ino の updateTurn 等）が絶対角度で行うため、
    // ここでは指定された向きに車輪速度目標をセットするだけでよい。
    mode_ = Mode::TURN;

    const float s = fabsf(speed_mps);
    if (target_angle_rad >= 0) {
        set_targets_mps(+s, -s);
    } else {
        set_targets_mps(-s, +s);
    }
}

void MotionController::stop() {
    mode_ = Mode::STOP;
    set_targets_mps(0.0f, 0.0f);
    pid_r_.reset();
    pid_l_.reset();
    motor_.set_right(0);
    motor_.set_left(0);
}

int16_t MotionController::calc_delta_14bit(uint16_t now, uint16_t prev) {
    int32_t d = static_cast<int32_t>(now) - static_cast<int32_t>(prev);
    d = (d % 16384 + 16384) % 16384;
    if (d > 8192) d -= 16384;
    return static_cast<int16_t>(d);
}

float MotionController::counts_to_m(int16_t delta_counts) {
    constexpr float WHEEL_DIAMETER_MM = 23.4f;
    constexpr float GEAR_RATIO = 1.0f;
    constexpr float ENCODER_RESOLUTION = 16384.0f;
    constexpr float COUNT_TO_MM = (WHEEL_DIAMETER_MM * 3.14159265359f) / (ENCODER_RESOLUTION * GEAR_RATIO);
    return (static_cast<float>(delta_counts) * COUNT_TO_MM) / 1000.0f;
}

void MotionController::set_targets_mps(float vr, float vl) {
    vr_ref_mps_ = vr;
    vl_ref_mps_ = vl;
}

void MotionController::apply_speed_pid(uint32_t dt_ms) {
    const float dt = static_cast<float>(dt_ms) / 1000.0f;

    if (vr_ref_mps_ == 0.0f && vl_ref_mps_ == 0.0f) {
        pid_r_.reset();
        pid_l_.reset();
        motor_.set_right(0);
        motor_.set_left(0);
        return;
    }

    const uint16_t r_angle = sensors_.get_right_wheel_angle();
    const uint16_t l_angle = sensors_.get_left_wheel_angle();

    if (!have_prev_) {
        prev_r_angle_ = r_angle;
        prev_l_angle_ = l_angle;
        have_prev_ = true;
        motor_.set_right(0);
        motor_.set_left(0);
        return;
    }

    const int16_t dr = calc_delta_14bit(r_angle, prev_r_angle_);
    const int16_t dl = calc_delta_14bit(l_angle, prev_l_angle_);
    prev_r_angle_ = r_angle;
    prev_l_angle_ = l_angle;

    // 前進方向の反転(2026-07-18)に合わせて符号を反転。sensors.cpp の
    // オドメトリ計算と揃えること。
    const float dist_r_m = counts_to_m(dr);
    const float dist_l_m = -counts_to_m(dl);

    const float vr_mps = (dt > 0) ? (dist_r_m / dt) : 0.0f;
    const float vl_mps = (dt > 0) ? (dist_l_m / dt) : 0.0f;

    // 量子化ノイズ対策のLPF(EMA)
    vr_filt_mps_ += SPEED_LPF_ALPHA * (vr_mps - vr_filt_mps_);
    vl_filt_mps_ += SPEED_LPF_ALPHA * (vl_mps - vl_filt_mps_);

    const float err_r = vr_ref_mps_ - vr_filt_mps_;
    const float err_l = vl_ref_mps_ - vl_filt_mps_;

    // フィードフォワード + PID補正（出力は duty: −1023〜+1023）
    float vbatt = sensors_.get_battery_voltage();
    if (vbatt < 6.0f) vbatt = VBATT_NOM;  // 起動直後・異常値ガード
    const float kf = KF_DUTY_PER_MPS * (VBATT_NOM / vbatt);
    float u_r = kf * vr_ref_mps_ + pid_r_.step(err_r, dt);
    float u_l = kf * vl_ref_mps_ + pid_l_.step(err_l, dt);
    if (u_r > 1023.0f) u_r = 1023.0f;
    if (u_r < -1023.0f) u_r = -1023.0f;
    if (u_l > 1023.0f) u_l = 1023.0f;
    if (u_l < -1023.0f) u_l = -1023.0f;

    // モーターに直接設定（D の Motor は −1023〜+1023 を受け入れる）
    last_duty_r_ = static_cast<int16_t>(u_r);
    last_duty_l_ = static_cast<int16_t>(u_l);
    motor_.set_right(last_duty_r_);
    motor_.set_left(last_duty_l_);
}
