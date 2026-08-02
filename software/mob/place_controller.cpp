#include "place_controller.h"
#include "params.h"
#include <math.h>

PlaceController::PlaceController(Motor& motor, Sensors& sensors)
    : motor_(motor), sensors_(sensors) {}

void PlaceController::reset_common() {
    have_prev_ = false;
    window_tick_count_ = 0;
    accum_dt_s_ = 0.0f;
    vr_filt_mps_ = 0.0f;
    vl_filt_mps_ = 0.0f;
    integ_ = 0.0f;
    prev_err_ = 0.0f;
    last_duty_ = 0;
    last_duty_diff_ = 0;
}

void PlaceController::start() {
    reset_common();
    turning_ = false;
    turn_omega_mag_ = 0.0f;
    turn_omega_signed_radps_ = 0.0f;
    turn_integ_ = 0.0f;
}

void PlaceController::start_turn(float delta_angle_rad) {
    reset_common();
    turning_ = true;
    turn_dir_ = (delta_angle_rad >= 0.0f) ? 1.0f : -1.0f;
    turn_start_angle_rad_ = sensors_.get_angle();
    turn_goal_angle_rad_ = turn_start_angle_rad_ + delta_angle_rad;
    turn_target_angle_rad_ = turn_start_angle_rad_;
    turn_omega_mag_ = 0.0f;
    turn_omega_signed_radps_ = 0.0f;
    turn_integ_ = 0.0f;
}

void PlaceController::stop() {
    motor_.set_right(0);
    motor_.set_left(0);
    window_tick_count_ = 0;
    accum_dt_s_ = 0.0f;
    integ_ = 0.0f;
    prev_err_ = 0.0f;
    last_duty_ = 0;
    last_duty_diff_ = 0;
    turning_ = false;
    turn_omega_mag_ = 0.0f;
    turn_integ_ = 0.0f;
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

float PlaceController::update_translational(float dt_s) {
    // 並進速度(左右輪速度の和)をゼロに保つPID。誤差が正(前へ流れている)
    // なら両輪へ同じ負のduty補正を、誤差が負(後ろへ流れている)なら
    // 両輪へ同じ正のduty補正を与える(左右差=回転成分には触れず、
    // 並進成分だけを対称に打ち消す)。start()単体でもstart_turn()と
    // 組み合わせても常にこの補正が動く。
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
    return u;
}

float PlaceController::update_turn_profile_and_track(float dt_s) {
    // 台形速度プロファイル: 目標角度(turn_target_angle_rad_)を、角加速度
    // params.pivot_accel一定で加速→params.pivot_max_radpsで巡航→
    // 同じ角加速度で減速、というプロファイルに沿って毎tick進める
    // (旧FWD/STOPコマンドの加減速プロファイルと同じ考え方)。
    const float total_angle = fabsf(turn_goal_angle_rad_ - turn_start_angle_rad_);
    const float traveled = fabsf(turn_target_angle_rad_ - turn_start_angle_rad_);
    const float remaining = total_angle - traveled;

    // 残り角度が「現在の角速度から一定角加速度で止まるのに必要な角度」
    // 以下になったら減速、それ以外は最大角速度まで加速(到達済みなら巡航)。
    const float decel_angle = (turn_omega_mag_ * turn_omega_mag_) / (2.0f * params.pivot_accel);
    float omega_next;
    if (remaining <= decel_angle) {
        omega_next = turn_omega_mag_ - params.pivot_accel * dt_s;
        if (omega_next < 0.0f) omega_next = 0.0f;
    } else {
        omega_next = turn_omega_mag_ + params.pivot_accel * dt_s;
        if (omega_next > params.pivot_max_radps) omega_next = params.pivot_max_radps;
    }
    turn_omega_mag_ = omega_next;

    float next_target = turn_target_angle_rad_ + turn_dir_ * turn_omega_mag_ * dt_s;
    // 目標を行き過ぎないようクランプ(丸め誤差で最後の1tickがオーバーする対策)
    if (turn_dir_ >= 0.0f) {
        if (next_target > turn_goal_angle_rad_) next_target = turn_goal_angle_rad_;
    } else {
        if (next_target < turn_goal_angle_rad_) next_target = turn_goal_angle_rad_;
    }
    turn_target_angle_rad_ = next_target;
    turn_omega_signed_radps_ = turn_dir_ * turn_omega_mag_;

    // 追従制御: フィードフォワード(目標角速度→duty、kf_duty_per_mpsと
    // 同じ考え方でwheel_base/2を掛けて車輪速度相当に換算) + 角度誤差の
    // PID + 角速度誤差(ジャイロ実測との差)のD的な補正。
    const float angle_error = turn_target_angle_rad_ - sensors_.get_angle();
    turn_integ_ += angle_error * dt_s;
    if (params.pivot_ki > 0.0f) {
        const float integ_max = params.pivot_out_max / params.pivot_ki;
        if (turn_integ_ > integ_max) turn_integ_ = integ_max;
        if (turn_integ_ < -integ_max) turn_integ_ = -integ_max;
    }
    const float rate_error = turn_omega_signed_radps_ - sensors_.get_gyro_z();

    float vbatt = sensors_.get_battery_voltage();
    if (vbatt < 6.0f) vbatt = params.vbatt_nom;  // 起動直後・異常値ガード
    const float kf = params.pivot_kf * (params.vbatt_nom / vbatt);

    float u = kf * turn_omega_signed_radps_
            + params.pivot_kp * angle_error
            + params.pivot_ki * turn_integ_
            + params.pivot_kd * rate_error;
    if (u > params.pivot_out_max) u = params.pivot_out_max;
    if (u < -params.pivot_out_max) u = -params.pivot_out_max;
    return u;
}

void PlaceController::update(float dt_s) {
    // 速度推定・制御更新はVELOCITY_WINDOW_TICKS(10ms)窓でまとめて行う
    // (ヘッダのコメント参照)。窓の途中は前回のduty(モーターは既に設定
    // 済み)を保持するだけで何もしない。
    accum_dt_s_ += dt_s;
    window_tick_count_++;
    if (window_tick_count_ < VELOCITY_WINDOW_TICKS) {
        return;
    }
    const float window_dt_s = accum_dt_s_;
    window_tick_count_ = 0;
    accum_dt_s_ = 0.0f;

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

    const float vr_mps = (window_dt_s > 0) ? (dist_r_m / window_dt_s) : 0.0f;
    const float vl_mps = (window_dt_s > 0) ? (dist_l_m / window_dt_s) : 0.0f;

    vr_filt_mps_ += params.place_lpf_alpha * (vr_mps - vr_filt_mps_);
    vl_filt_mps_ += params.place_lpf_alpha * (vl_mps - vl_filt_mps_);

    const float duty_common = update_translational(window_dt_s);
    const float duty_diff = turning_ ? update_turn_profile_and_track(window_dt_s) : 0.0f;

    last_duty_ = static_cast<int16_t>(duty_common);
    last_duty_diff_ = static_cast<int16_t>(duty_diff);

    float u_r = duty_common + duty_diff;
    float u_l = duty_common - duty_diff;
    if (u_r > 1023.0f) u_r = 1023.0f;
    if (u_r < -1023.0f) u_r = -1023.0f;
    if (u_l > 1023.0f) u_l = 1023.0f;
    if (u_l < -1023.0f) u_l = -1023.0f;

    motor_.set_right(static_cast<int16_t>(u_r));
    motor_.set_left(static_cast<int16_t>(u_l));
}
