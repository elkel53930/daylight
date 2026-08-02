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
    last_duty_common_f_ = 0.0f;
    last_duty_diff_f_ = 0.0f;
    accel_filt_mps2_ = 0.0f;
}

void PlaceController::start() {
    reset_common();
    pos_ref_mm_ = sensors_.get_distance();
    turning_ = false;
    turn_omega_mag_ = 0.0f;
    turn_omega_signed_radps_ = 0.0f;
    turn_integ_ = 0.0f;
}

void PlaceController::start_turn(float delta_angle_rad) {
    reset_common();
    pos_ref_mm_ = sensors_.get_distance();
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
    last_duty_common_f_ = 0.0f;
    last_duty_diff_f_ = 0.0f;
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
    // 外側ループ: 位置(sensors.get_distance())を開始時の基準へ戻すP制御で
    // 目標並進速度を作る。速度(左右輪速度の和)の平均をゼロにするだけでは
    // 復元力が無く、位置がゆっくりドリフトしうるため(2026-08-02追加)。
    const float pos_error_m = (pos_ref_mm_ - sensors_.get_distance()) / 1000.0f;
    float target_v_sum = params.place_pos_kp * pos_error_m;
    if (target_v_sum > params.place_pos_max_mps) target_v_sum = params.place_pos_max_mps;
    if (target_v_sum < -params.place_pos_max_mps) target_v_sum = -params.place_pos_max_mps;

    // 内側ループ: 並進速度(左右輪速度の和)を上記の目標値へ追い込むPID。
    // 誤差が正(目標より遅れている/後ろへ流れている)なら両輪へ同じ正の
    // duty補正を与える(左右差=回転成分には触れず、並進成分だけを対称に
    // 補正する)。start()単体でもstart_turn()と組み合わせても常に動く。
    const float v_sum = vr_filt_mps_ + vl_filt_mps_;
    const float err = target_v_sum - v_sum;

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
    // 毎ms: IMU Y軸(ロボット前後方向、+=前進側)加速度によるフィード
    // フォワード減衰。エンコーダ差分ベースの速度PID(10ms窓)より速く
    // 外乱に反応できる(2026-08-02追加)。u_accel_ffの符号は、前へ加速
    // (accel>0)している間は両輪へ負のduty補正を与えて打ち消す向き。
    //
    // 旋回中(turning_)は無効化する: IMUは回転中心からオフセットして
    // 搭載されているため、旋回時は向心加速度(ω^2×r)・接線加速度(α×r)
    // がY軸にも混入し、実際には並進していないのに大きな値を示す。これを
    // そのまま並進外乱として補正しようとした結果、実機で大暴走した
    // (2026-08-02、vsumが±50〜70mm/sまで発振・duty上限近くまで振れた)。
    const float accel_mps2 = sensors_.get_accel_forward();
    accel_filt_mps2_ += params.place_accel_lpf_alpha * (accel_mps2 - accel_filt_mps2_);
    const float duty_accel_ff = turning_ ? 0.0f : (-params.place_accel_kd * accel_filt_mps2_);

    // 速度推定・並進/旋回PIDの更新はVELOCITY_WINDOW_TICKS(10ms)窓でまとめて
    // 行う(ヘッダのコメント参照)。窓の途中は前回計算したduty_common/
    // duty_diffを保持したまま、加速度FFだけ毎tick更新して出力する。
    accum_dt_s_ += dt_s;
    window_tick_count_++;
    if (window_tick_count_ >= VELOCITY_WINDOW_TICKS) {
        const float window_dt_s = accum_dt_s_;
        window_tick_count_ = 0;
        accum_dt_s_ = 0.0f;

        const uint16_t r_angle = sensors_.get_right_wheel_angle();
        const uint16_t l_angle = sensors_.get_left_wheel_angle();

        if (!have_prev_) {
            prev_r_angle_ = r_angle;
            prev_l_angle_ = l_angle;
            have_prev_ = true;
            last_duty_common_f_ = 0.0f;
            last_duty_diff_f_ = 0.0f;
            last_duty_ = 0;
            last_duty_diff_ = 0;
        } else {
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

            last_duty_common_f_ = update_translational(window_dt_s);
            last_duty_diff_f_ = turning_ ? update_turn_profile_and_track(window_dt_s) : 0.0f;
            last_duty_ = static_cast<int16_t>(last_duty_common_f_);
            last_duty_diff_ = static_cast<int16_t>(last_duty_diff_f_);
        }
    }

    float u_r = last_duty_common_f_ + last_duty_diff_f_ + duty_accel_ff;
    float u_l = last_duty_common_f_ - last_duty_diff_f_ + duty_accel_ff;
    if (u_r > 1023.0f) u_r = 1023.0f;
    if (u_r < -1023.0f) u_r = -1023.0f;
    if (u_l > 1023.0f) u_l = 1023.0f;
    if (u_l < -1023.0f) u_l = -1023.0f;

    motor_.set_right(static_cast<int16_t>(u_r));
    motor_.set_left(static_cast<int16_t>(u_l));
}
