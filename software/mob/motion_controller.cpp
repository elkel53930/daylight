#include "motion_controller.h"
#include "params.h"
#include <math.h>

// チューニング定数は params.h/params.cpp の Params 構造体(実行時に
// PSET/PSAVEで機体ごとに変更・永続化できる)にある。
//
// params.speed_kp/speed_ki/speed_kd: 車輪速度PID。
// 実測プラントゲイン ≈ 3mm/s per duty (2026-07-19 テレメトリ: duty96↔300mm/s)。
// params.kf_duty_per_mps/vbatt_nom: 速度フィードフォワード: 目標速度[m/s]→duty。
// 実効ゲインは電圧に反比例するため vbatt でスケーリングする(電圧変動で
// duty↔速度の関係が±10%以上ずれるのを補正)。
// params.speed_lpf_alpha: 速度測定(1ms毎のエンコーダ差分)の量子化ノイズを
// 抑えるLPF係数。
//
// 2026-08-02: 距離・角度プロファイルを持つ上位制御(旧FWD/STOP/TURN等)と、
// それに付随していた旋回中の左右速度同期補正を削除し、MOT/DUTY用の素の
// 車輪速度PIDだけに整理した(旧実装の詳細はgit履歴参照)。

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
    // kp/ki/kd の初期値はここで一度だけ設定するが、実行中に PSET で
    // params.speed_kp 等が変わった場合は apply_speed_pid() で毎回
    // 反映し直す(integ/prev_err は保持したままゲインだけ更新する)。
    pid_r_ = {params.speed_kp, params.speed_ki, params.speed_kd, 0.0f, 0.0f, -1023.0f, 1023.0f};
    pid_l_ = {params.speed_kp, params.speed_ki, params.speed_kd, 0.0f, 0.0f, -1023.0f, 1023.0f};
}

void MotionController::update(uint32_t dt_ms) {
    apply_speed_pid(dt_ms);
}

void MotionController::forward(float speed_mps) {
    mode_ = Mode::FORWARD;
    set_targets_mps(speed_mps, speed_mps);
}

void MotionController::set_duty_direct(int16_t r_duty, int16_t l_duty) {
    mode_ = Mode::DUTY_DIRECT;

    if (r_duty > 1023) r_duty = 1023;
    if (r_duty < -1023) r_duty = -1023;
    if (l_duty > 1023) l_duty = 1023;
    if (l_duty < -1023) l_duty = -1023;

    last_duty_r_ = r_duty;
    last_duty_l_ = l_duty;
    motor_.set_right(last_duty_r_);
    motor_.set_left(last_duty_l_);

    pid_r_.reset();
    pid_l_.reset();
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

    // PSET で変更されている可能性があるゲインを毎回反映する
    // (integ/prev_err はリセットしない)。
    pid_r_.kp = params.speed_kp;
    pid_r_.ki = params.speed_ki;
    pid_r_.kd = params.speed_kd;
    pid_l_.kp = params.speed_kp;
    pid_l_.ki = params.speed_ki;
    pid_l_.kd = params.speed_kd;

    // エンコーダによる速度推定は、モード(速度PID経由かDUTY_DIRECTか)に
    // 関わらず常に行う。DUTY_DIRECT中もテレメトリ(#Vのvr/vl)を意味の
    // あるものにするため。
    const uint16_t r_angle = sensors_.get_right_wheel_angle();
    const uint16_t l_angle = sensors_.get_left_wheel_angle();

    if (!have_prev_) {
        prev_r_angle_ = r_angle;
        prev_l_angle_ = l_angle;
        have_prev_ = true;
        if (mode_ != Mode::DUTY_DIRECT) {
            motor_.set_right(0);
            motor_.set_left(0);
        }
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
    vr_filt_mps_ += params.speed_lpf_alpha * (vr_mps - vr_filt_mps_);
    vl_filt_mps_ += params.speed_lpf_alpha * (vl_mps - vl_filt_mps_);

    if (mode_ == Mode::DUTY_DIRECT) {
        // duty は set_duty_direct() が既に直接設定済み。速度PIDは経由しない。
        return;
    }

    if (vr_ref_mps_ == 0.0f && vl_ref_mps_ == 0.0f) {
        pid_r_.reset();
        pid_l_.reset();
        motor_.set_right(0);
        motor_.set_left(0);
        return;
    }

    const float err_r = vr_ref_mps_ - vr_filt_mps_;
    const float err_l = vl_ref_mps_ - vl_filt_mps_;

    // フィードフォワード + PID補正（出力は duty: −1023〜+1023）
    float vbatt = sensors_.get_battery_voltage();
    if (vbatt < 6.0f) vbatt = params.vbatt_nom;  // 起動直後・異常値ガード
    const float kf = params.kf_duty_per_mps * (params.vbatt_nom / vbatt);
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
