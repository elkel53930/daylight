#include "motion_controller.h"
#include "params.h"
#include <math.h>

// チューニング定数は params.h/params.cpp の Params 構造体(実行時に
// PSET/PSAVEで機体ごとに変更・永続化できる)に移動した。以下は各値の
// 意味・調整経緯の記録(ビルド時デフォルトは params.cpp の
// kDefaultParams 参照)。
//
// params.speed_kp/speed_ki/speed_kd: 車輪速度PID。
// 実測プラントゲイン ≈ 3mm/s per duty (2026-07-19 テレメトリ: duty96↔300mm/s)。
// KP=800はループゲイン≈2.4で振動していたため、FF主体+P縮小(≈0.9)に変更。
// KI=60 は弱すぎて実質P制御になり、負荷・電圧変動がそのまま速度変動に
// 現れていた(2026-07-19)。定常dutyを積分項が数百msで担える値に引き上げ。
//
// params.kf_duty_per_mps/vbatt_nom: 速度フィードフォワード: 目標速度[m/s]→duty。
// vbatt_nom 時の値。実効ゲインは電圧に反比例するため vbatt でスケーリングする
// (2026-07-19: 計測毎に duty↔速度 の関係が±10%以上ずれる原因が電圧変動だった)。
// 定常テレメトリ実測 duty100 ↔ 約370mm/s(vbatt≈7.5V)から逆算。
//
// params.speed_lpf_alpha: 速度測定(1ms毎のエンコーダ差分)の量子化ノイズを
// 抑えるLPF係数。0.3では±100mm/s級のスパイク(ジャイロと矛盾=計測ノイズ)
// が残った(2026-07-19)。
//
// 旋回中の左右速度同期(2026-07-22追加、2回無効化: 2026-07-22)。
// 左右輪は独立したPID(同一ゲイン・同一目標速度)で追従しているが、
// 摩擦やギアの個体差で実速度がずれると旋回中心が機械的中心からずれる
// (並進成分が生じる)。turn_in_place() が毎tick与える目標(±s、同じ大きさ)
// はそのままに、実測速度の絶対値の差だけを打ち消す向きに左右の目標を
// 逆方向へ微調整して追従させる…つもりだった。
//
// 1回目(導入直後、条件付きPID化と同時テスト): オーバーシュート悪化
// (90°で-18〜-24°、180°で-12〜-13°) → ゲイン0で無効化。
// TURN_ACCEL_MPS2・TURN_MAX_SPEED_MPS(0.25→0.15)の調整でオーバーシュート
// 自体が一桁台まで縮小したため、干渉する土壌が薄れたと考え再有効化
// (クランプもTURN_MAX_SPEED_MPSに比例縮小: 0.08→0.05)。
// 2回目(このゲインで再テスト): それでも悪化(右90°6.6°→9.9°・3回
// リトライ、180°3.6°→9.7°・2回リトライ)。ゲインを弱めても改善せず、
// 個別車輪PID(KP=300,KI=3000)と同じ雑音混じりの実測速度に反応する
// 同期ループという設計自体が振動源になっていると判断し、再度無効化。
// 左右差対策として別のアプローチ(例: 十分にLPFした差分を使う、応答を
// もっと遅くする等)が必要。
//
// 3回目(2026-07-24、右モーター軸のギア滑り修理後に再挑戦)。過去2回の
// 振動は「同期ループが個別車輪PIDと同じ生の速度信号(SPEED_LPF_ALPHA=
// 0.12、時定数≈8ms)に反応し結合していた」ことが原因と考え、同期誤差
// 専用に別の緩いLPF(SYNC_ERR_LPF_ALPHA、時定数≈50ms)をかけて低周波の
// 定常ずれ(ギア・摩擦の個体差)だけに反応させ、車輪PIDの速い応答とは
// 周波数帯で分離する。ゲインは小さめから開始し実機で追い込む。
// params.turn_sync_kp: 無次元(m/s差 → m/s補正)
// params.turn_sync_ki: [1/s]
// params.sync_max_corr: 補正量クランプ(グリッチ対策)
// params.sync_lpf_alpha: 同期誤差専用LPFの係数。speed_lpf_alpha(0.12)
// より十分小さくし、個別車輪PIDと反応する周波数帯が重ならないようにする。

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

void MotionController::forward(float speed_mps, float lateral_error) {
    mode_ = Mode::FORWARD;

    const float corr = k_lateral_ * lateral_error;
    set_targets_mps(speed_mps - corr, speed_mps + corr);
}

void MotionController::backward(float speed_mps, float lateral_error) {
    mode_ = Mode::BACKWARD;
    // yaw_rate は (vr-vl) に線形なので、forward() と同じ符号の補正で
    // (基準速度が負であっても)同じ向きに効く。lateral_error 未指定
    // (既定0.0)なら従来と同じ左右等速。
    const float corr = k_lateral_ * lateral_error;
    const float s = -fabsf(speed_mps);
    set_targets_mps(s - corr, s + corr);
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
    turn_sync_integ_ = 0.0f;
    turn_sync_err_filt_ = 0.0f;
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
        turn_sync_integ_ = 0.0f;
        turn_sync_err_filt_ = 0.0f;
        motor_.set_right(0);
        motor_.set_left(0);
        return;
    }

    // 旋回中のみ: 実測速度の絶対値を左右で揃える同期補正。
    // turn_in_place() が設定した ±s（同じ大きさ）はそのままに、
    // 「速い方を減速・遅い方を増速」させて中心の並進成分を抑える。
    float vr_ref_eff = vr_ref_mps_;
    float vl_ref_eff = vl_ref_mps_;
    if (mode_ == Mode::TURN) {
        const float mag_r = fabsf(vr_filt_mps_);
        const float mag_l = fabsf(vl_filt_mps_);
        const float sync_err_raw = mag_r - mag_l;  // +: 右が速い

        // 個別車輪PID(速い応答)と結合して振動しないよう、同期誤差は
        // さらに緩いLPFを通してから使う(低周波の定常ずれのみ反応)。
        turn_sync_err_filt_ += params.sync_lpf_alpha * (sync_err_raw - turn_sync_err_filt_);
        const float sync_err = turn_sync_err_filt_;

        turn_sync_integ_ += sync_err * dt;
        if (params.turn_sync_ki > 0.0f) {
            const float sync_integ_max = params.sync_max_corr / params.turn_sync_ki;
            if (turn_sync_integ_ > sync_integ_max) turn_sync_integ_ = sync_integ_max;
            if (turn_sync_integ_ < -sync_integ_max) turn_sync_integ_ = -sync_integ_max;
        }

        float sync_corr = params.turn_sync_kp * sync_err + params.turn_sync_ki * turn_sync_integ_;
        if (sync_corr > params.sync_max_corr) sync_corr = params.sync_max_corr;
        if (sync_corr < -params.sync_max_corr) sync_corr = -params.sync_max_corr;

        const float dir_r = (vr_ref_mps_ >= 0.0f) ? 1.0f : -1.0f;
        const float dir_l = (vl_ref_mps_ >= 0.0f) ? 1.0f : -1.0f;
        vr_ref_eff = dir_r * (fabsf(vr_ref_mps_) - 0.5f * sync_corr);
        vl_ref_eff = dir_l * (fabsf(vl_ref_mps_) + 0.5f * sync_corr);
    } else {
        turn_sync_integ_ = 0.0f;
        turn_sync_err_filt_ = 0.0f;
    }

    const float err_r = vr_ref_eff - vr_filt_mps_;
    const float err_l = vl_ref_eff - vl_filt_mps_;

    // フィードフォワード + PID補正（出力は duty: −1023〜+1023）
    float vbatt = sensors_.get_battery_voltage();
    if (vbatt < 6.0f) vbatt = params.vbatt_nom;  // 起動直後・異常値ガード
    const float kf = params.kf_duty_per_mps * (params.vbatt_nom / vbatt);
    float u_r = kf * vr_ref_eff + pid_r_.step(err_r, dt);
    float u_l = kf * vl_ref_eff + pid_l_.step(err_l, dt);
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
