#include "place_controller.h"
#include "params.h"

PlaceController::PlaceController(Motor& motor, Sensors& sensors)
    : motor_(motor), sensors_(sensors) {}

void PlaceController::start() {
    have_prev_ = false;
    vr_filt_mps_ = 0.0f;
    vl_filt_mps_ = 0.0f;
    integ_ = 0.0f;
    prev_err_ = 0.0f;
    last_duty_ = 0;
}

void PlaceController::stop() {
    motor_.set_right(0);
    motor_.set_left(0);
    integ_ = 0.0f;
    prev_err_ = 0.0f;
    last_duty_ = 0;
}

int16_t PlaceController::calc_delta_14bit(uint16_t now, uint16_t prev) {
    int32_t d = static_cast<int32_t>(now) - static_cast<int32_t>(prev);
    d = (d % 16384 + 16384) % 16384;
    if (d > 8192) d -= 16384;
    return static_cast<int16_t>(d);
}

float PlaceController::counts_to_m(int16_t delta_counts) {
    // motion_controller.cpp::counts_to_m() と同じ換算(ホイール直径・
    // ギア比・エンコーダ分解能はsensors.h/motion_controller.cppに合わせる)。
    constexpr float WHEEL_DIAMETER_MM = 23.4f;
    constexpr float GEAR_RATIO = 1.0f;
    constexpr float ENCODER_RESOLUTION = 16384.0f;
    constexpr float COUNT_TO_MM = (WHEEL_DIAMETER_MM * 3.14159265359f) / (ENCODER_RESOLUTION * GEAR_RATIO);
    return (static_cast<float>(delta_counts) * COUNT_TO_MM) / 1000.0f;
}

void PlaceController::update(float dt_s) {
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

    // 前進方向の符号規約はmotion_controller.cppと同じ(左輪のみ反転)。
    const float dist_r_m = counts_to_m(dr);
    const float dist_l_m = -counts_to_m(dl);

    const float vr_mps = (dt_s > 0) ? (dist_r_m / dt_s) : 0.0f;
    const float vl_mps = (dt_s > 0) ? (dist_l_m / dt_s) : 0.0f;

    vr_filt_mps_ += params.place_lpf_alpha * (vr_mps - vr_filt_mps_);
    vl_filt_mps_ += params.place_lpf_alpha * (vl_mps - vl_filt_mps_);

    // 並進速度(左右輪速度の和)をゼロに保つPID。誤差が正(前へ流れている)
    // なら両輪へ同じ負のduty補正を、誤差が負(後ろへ流れている)なら
    // 両輪へ同じ正のduty補正を与える(左右差=回転成分には触れず、
    // 並進成分だけを対称に打ち消す)。
    const float v_sum = vr_filt_mps_ + vl_filt_mps_;
    const float err = 0.0f - v_sum;

    integ_ += err * dt_s;
    if (params.place_ki > 0.0f) {
        const float integ_max = params.place_out_max / params.place_ki;
        if (integ_ > integ_max) integ_ = integ_max;
        if (integ_ < -integ_max) integ_ = -integ_max;
    }
    const float deriv = (dt_s > 0) ? (err - prev_err_) / dt_s : 0.0f;
    prev_err_ = err;

    float u = params.place_kp * err + params.place_ki * integ_ + params.place_kd * deriv;
    if (u > params.place_out_max) u = params.place_out_max;
    if (u < -params.place_out_max) u = -params.place_out_max;

    last_duty_ = static_cast<int16_t>(u);
    motor_.set_right(last_duty_);
    motor_.set_left(last_duty_);
}
