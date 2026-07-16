#include "motion_controller.h"
#include <math.h>

// Tuning constants (initial / placeholder)
static constexpr float DEFAULT_KP = 800.0f;
static constexpr float DEFAULT_KI = 60.0f;
static constexpr float DEFAULT_KD = 0.0f;

float MotionController::SpeedPID::step(float err, float dt_s) {
    if (dt_s <= 0) return 0.0f;
    integ += err * dt_s;
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

bool MotionController::turn_in_place(float speed_mps, float target_angle_rad) {
    if (!turn_active_) {
        turn_active_ = true;
        turn_target_rad_ = target_angle_rad;
        turn_start_angle_rad_ = sensors_.get_angle();
    }

    mode_ = Mode::TURN;

    const float s = fabsf(speed_mps);
    if (target_angle_rad >= 0) {
        set_targets_mps(+s, -s);
    } else {
        set_targets_mps(-s, +s);
    }

    const float current = sensors_.get_angle();
    const float turned = current - turn_start_angle_rad_;
    if ((target_angle_rad >= 0 && turned >= target_angle_rad) ||
        (target_angle_rad < 0 && turned <= target_angle_rad)) {
        stop();
        turn_active_ = false;
        return true;
    }

    return false;
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
    constexpr float GEAR_RATIO = 41.0f / 20.0f;
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

    const float dist_r_m = -counts_to_m(dr);
    const float dist_l_m = counts_to_m(dl);

    const float vr_mps = (dt > 0) ? (dist_r_m / dt) : 0.0f;
    const float vl_mps = (dt > 0) ? (dist_l_m / dt) : 0.0f;

    const float err_r = vr_ref_mps_ - vr_mps;
    const float err_l = vl_ref_mps_ - vl_mps;

    // PID出力（これは「speed」単位：−1023〜+1023）
    float u_r = pid_r_.step(err_r, dt);
    float u_l = pid_l_.step(err_l, dt);

    // モーターに直接設定（D の Motor は −1023〜+1023 を受け入れる）
    motor_.set_right(static_cast<int16_t>(u_r));
    motor_.set_left(static_cast<int16_t>(u_l));
}
