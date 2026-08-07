#include "place_controller.h"
#include "params.h"
#include <math.h>

// JOG(JOGFWD/JOGBACK/JOGTURN)の到達・整定判定しきい値。到達=目標に十分近い
// かつ十分止まっている、で完了通知(DONE)する。5mm許容の運用ルールに対し
// 余裕を持たせた小さめの値。
static constexpr float JOG_DIST_TOL_MM = 1.5f;      // 位置到達許容[mm]
static constexpr float JOG_VEL_TOL_MPS = 0.010f;    // 並進速度(vr+vl)の整定許容[m/s]
static constexpr float JOG_ANG_TOL_RAD = 0.010f;    // 角度到達許容[rad](≈0.57deg)
static constexpr float JOG_RATE_TOL_RADPS = 0.05f;  // 角速度の整定許容[rad/s]

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
    last_sync_r_f_ = 0.0f;
    last_sync_l_f_ = 0.0f;
    sync_err_filt_ = 0.0f;
    accel_filt_mps2_ = 0.0f;
}

void PlaceController::start() {
    reset_common();
    pos_ref_mm_ = sensors_.get_distance();
    turning_ = false;
    turn_omega_mag_ = 0.0f;
    turn_omega_signed_radps_ = 0.0f;
    turn_integ_ = 0.0f;
    jog_kind_ = JogKind::NONE;
    jog_arrived_latch_ = false;
}

void PlaceController::start_turn(float delta_angle_rad, bool as_jog) {
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
    jog_kind_ = as_jog ? JogKind::TURN : JogKind::NONE;
    jog_arrived_latch_ = false;
}

void PlaceController::start_move(float delta_dist_mm) {
    reset_common();
    // 並進側: pos_ref を delta ぶんずらす。update_translational の位置P制御が
    // place_pos_max_mps で低速クランプしつつ滑らかに寄せ、到達後は保持する。
    pos_ref_mm_ = sensors_.get_distance() + delta_dist_mm;
    // 旋回側: 現在の向きを保持(delta=0のstart_turn相当)。移動中もヨーを
    // 開始角に保つことで、直進が横へ逸れるのを防ぐ。
    turning_ = true;
    turn_dir_ = 1.0f;
    turn_start_angle_rad_ = sensors_.get_angle();
    turn_goal_angle_rad_ = turn_start_angle_rad_;
    turn_target_angle_rad_ = turn_start_angle_rad_;
    turn_omega_mag_ = 0.0f;
    turn_omega_signed_radps_ = 0.0f;
    turn_integ_ = 0.0f;
    jog_kind_ = JogKind::MOVE;
    jog_arrived_latch_ = false;
}

bool PlaceController::take_jog_arrived() {
    if (jog_arrived_latch_) {
        jog_arrived_latch_ = false;
        return true;
    }
    return false;
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
    last_sync_r_f_ = 0.0f;
    last_sync_l_f_ = 0.0f;
    sync_err_filt_ = 0.0f;
    turning_ = false;
    turn_omega_mag_ = 0.0f;
    turn_integ_ = 0.0f;
    jog_kind_ = JogKind::NONE;
    jog_arrived_latch_ = false;
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
    constexpr float WHEEL_DIAMETER_MM = 23.0f;
    constexpr float GEAR_RATIO = 1.0f;
    constexpr float ENCODER_RESOLUTION = 16384.0f;
    constexpr float COUNT_TO_MM = (WHEEL_DIAMETER_MM * 3.14159265359f) / (ENCODER_RESOLUTION * GEAR_RATIO);
    return (static_cast<float>(delta_counts) * COUNT_TO_MM) / 1000.0f;
}

float PlaceController::update_translational(float dt_s) {
    // 外側ループ: 位置(sensors.get_distance())から目標並進速度を作る。
    // JOGFWD/JOGBACK は移動用ゲイン、HOLD/TURN/JOGTURN は並進抑制専用
    // の弱めゲインを使い分ける(2026-08-05)。
    const bool jog_move = (jog_kind_ == JogKind::MOVE);
    const float pos_kp = jog_move ? params.place_pos_kp : params.place_hold_pos_kp;
    // JOGFWD/JOGBACK のみ、位置外側ループの速度上限を2倍にして実速度を引き上げる。
    const float pos_max = jog_move ? (params.place_pos_max_mps * 2.0f) : params.place_hold_pos_max_mps;
    const float pid_kp = jog_move ? params.place_kp : params.place_hold_kp;
    const float pid_ki = jog_move ? params.place_ki : params.place_hold_ki;
    const float pid_kd = jog_move ? params.place_kd : params.place_hold_kd;
    const float pid_out_max = jog_move ? params.place_out_max : params.place_hold_out_max;

    const float pos_error_m = (pos_ref_mm_ - sensors_.get_distance()) / 1000.0f;
    float target_v_sum = pos_kp * pos_error_m;
    if (target_v_sum > pos_max) target_v_sum = pos_max;
    if (target_v_sum < -pos_max) target_v_sum = -pos_max;

    // 内側ループ: 並進速度(左右輪速度の和)を上記の目標値へ追い込むPID。
    // 誤差が正(目標より遅れている/後ろへ流れている)なら両輪へ同じ正の
    // duty補正を与える(左右差=回転成分には触れず、並進成分だけを対称に
    // 補正する)。start()単体でもstart_turn()と組み合わせても常に動く。
    const float v_sum = vr_filt_mps_ + vl_filt_mps_;
    const float err = target_v_sum - v_sum;

    integ_ += err * dt_s;
    if (pid_ki > 0.0f) {
        const float integ_max = pid_out_max / pid_ki;
        if (integ_ > integ_max) integ_ = integ_max;
        if (integ_ < -integ_max) integ_ = -integ_max;
    }
    const float deriv = (dt_s > 0) ? (err - prev_err_) / dt_s : 0.0f;
    prev_err_ = err;

    float u = pid_kp * err + pid_ki * integ_ + pid_kd * deriv;
    if (u > pid_out_max) u = pid_out_max;
    if (u < -pid_out_max) u = -pid_out_max;
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

void PlaceController::update_wheel_sync(float dt_s) {
    (void)dt_s;
    // 左右輪速度の"大きさ"(向きは問わない)をP制御で揃える。duty_common
    // (並進、左右へ同じ値)・duty_diff(旋回、左右へ逆符号)はどちらも
    // 「左右のモーターが指令に対して同じように応答する」前提だが、
    // 実際はモーター個体差で同じdutyでも実速度が揃わないことがある。
    // その場合、たとえvsum(和)がゼロでも機体はゆっくり回頭してしまい
    // (並進側の制御からは検出できない位置ズレの原因になる)、旋回中は
    // 旋回中心が機械的中心からずれる。速い方を弱め遅い方を強めることで
    // 打ち消す(2026-08-02追加、ユーザー指摘によりP制御のみで追加)。
    const float mag_r = fabsf(vr_filt_mps_);
    const float mag_l = fabsf(vl_filt_mps_);
    const float sync_err_raw = mag_r - mag_l;  // +: 右が速い

    // 車輪速度PID/並進制御(place_lpf_alpha≈0.12=速い応答)と同じ生の速度信号に
    // 反応すると結合して振動し、旋回中に前後発振→横drift になる。旧
    // motion_controller で実証・解決済(c816154): 同期誤差専用の緩いLPF
    // (place_sync_err_lpf_alpha≈0.02≈50ms)を通し、低周波の定常ずれ(ギア・
    // 摩擦の個体差)だけに反応させて車輪PIDと周波数帯を分離する。
    sync_err_filt_ += params.place_sync_err_lpf_alpha * (sync_err_raw - sync_err_filt_);
    const float sync_err = sync_err_filt_;

    float adj = params.place_sync_kp * sync_err;
    if (adj > params.place_sync_out_max) adj = params.place_sync_out_max;
    if (adj < -params.place_sync_out_max) adj = -params.place_sync_out_max;

    const float sign_r = (vr_filt_mps_ >= 0.0f) ? 1.0f : -1.0f;
    const float sign_l = (vl_filt_mps_ >= 0.0f) ? 1.0f : -1.0f;
    last_sync_r_f_ = -sign_r * adj;  // 右が速ければ右自身の向きと逆に効かせ弱める
    last_sync_l_f_ = sign_l * adj;   // 右が速ければ左自身の向きへ強めて追いつかせる
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
            last_sync_r_f_ = 0.0f;
            last_sync_l_f_ = 0.0f;
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
            update_wheel_sync(window_dt_s);
            last_duty_ = static_cast<int16_t>(last_duty_common_f_);
            last_duty_diff_ = static_cast<int16_t>(last_duty_diff_f_);
        }
    }

    float u_r = last_duty_common_f_ + last_duty_diff_f_ + duty_accel_ff + last_sync_r_f_;
    float u_l = last_duty_common_f_ - last_duty_diff_f_ + duty_accel_ff + last_sync_l_f_;
    if (u_r > 1023.0f) u_r = 1023.0f;
    if (u_r < -1023.0f) u_r = -1023.0f;
    if (u_l > 1023.0f) u_l = 1023.0f;
    if (u_l < -1023.0f) u_l = -1023.0f;

    motor_.set_right(static_cast<int16_t>(u_r));
    motor_.set_left(static_cast<int16_t>(u_l));

    // JOG完了判定: 目標へ到達し十分止まったら arrived ラッチを立て、以降は
    // 通常の保持へ移す(kind=NONE、pos_ref/turn_goalはそのままなので位置・
    // 向きの保持は継続する)。開始直後は目標まで距離/角度があり誤検出しない。
    if (jog_kind_ == JogKind::MOVE) {
        const float pos_err_mm = fabsf(pos_ref_mm_ - sensors_.get_distance());
        const float v_sum = fabsf(vr_filt_mps_ + vl_filt_mps_);
        if (pos_err_mm < JOG_DIST_TOL_MM && v_sum < JOG_VEL_TOL_MPS) {
            jog_arrived_latch_ = true;
            jog_kind_ = JogKind::NONE;
        }
    } else if (jog_kind_ == JogKind::TURN) {
        const float ang_err = fabsf(turn_goal_angle_rad_ - sensors_.get_angle());
        const float rate = fabsf(sensors_.get_gyro_z());
        if (ang_err < JOG_ANG_TOL_RAD && rate < JOG_RATE_TOL_RADPS) {
            jog_arrived_latch_ = true;
            jog_kind_ = JogKind::NONE;
        }
    }
}
