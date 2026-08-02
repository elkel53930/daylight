#include "path_controller.h"
#include "params.h"
#include <math.h>

namespace {
constexpr float PI_F = 3.14159265359f;

float normalize_angle(float a) {
    while (a > PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}
}  // namespace

PathController::PathController(Motor& motor, Sensors& sensors)
    : motor_(motor), sensors_(sensors) {}

void PathController::start(const Segment* segments, size_t count) {
    // start()時点でセグメントを内部バッファへコピー(スナップショット)する。
    // 呼び出し側(mob.ino)は走行コマンドをCore1で直接g_pattern_segmentsへ
    // 積むため、走行中にそのバッファが書き換わっても現在の走行に影響
    // しないよう、ここで独立した控えを持つ(2026-08-02)。
    if (count > MAX_SEGMENTS) count = MAX_SEGMENTS;
    for (size_t i = 0; i < count; i++) {
        segments_storage_[i] = segments[i];
    }
    segments_ = segments_storage_;
    seg_count_ = count;
    seg_index_ = 0;

    start_heading_rad_ = sensors_.get_angle();
    prev_sensors_dist_mm_ = sensors_.get_distance();

    robot_x_mm_ = 0.0f;
    robot_y_mm_ = 0.0f;
    robot_theta_rad_ = 0.0f;

    // 仮想ターゲットの初期位置: ロボット前方 path_follow_mm(既定30mm)。
    target_x_mm_ = params.path_follow_mm;
    target_y_mm_ = 0.0f;
    target_heading_rad_ = 0.0f;
    target_speed_mmps_ = 0.0f;

    omega_ff_radps_ = 0.0f;
    dist_to_target_mm_ = params.path_follow_mm;
    heading_error_rad_ = 0.0f;

    if (seg_count_ > 0) {
        begin_segment(0);
    }
}

void PathController::stop() {
    motor_.set_right(0);
    motor_.set_left(0);
    seg_index_ = seg_count_;  // 残りセグメントは破棄する(再開はstart()から)
    target_speed_mmps_ = 0.0f;
    omega_ff_radps_ = 0.0f;
}

void PathController::begin_segment(size_t index) {
    seg_index_ = index;
    seg_start_x_mm_ = target_x_mm_;
    seg_start_y_mm_ = target_y_mm_;
    seg_start_heading_rad_ = target_heading_rad_;
    seg_progress_ = 0.0f;

    const Segment& seg = segments_[index];
    if (seg.type == SegmentType::STRAIGHT) {
        target_speed_mmps_ = seg.v_start_mmps;
        omega_ff_radps_ = 0.0f;
    } else {
        target_speed_mmps_ = seg.v_mmps;
        omega_ff_radps_ = seg.dir * seg.v_mmps / seg.radius_mm;
    }
}

void PathController::advance_straight(const Segment& seg, float dt_s) {
    const float remaining = seg.distance_mm - seg_progress_;

    // 台形速度プロファイル(旧FWD/STOPコマンド・その場旋回プロファイルと
    // 同じ考え方): 残り距離が「現在速度からv_endまで減速(加速)するのに
    // 必要な距離」以下になったらv_endへ向け、それ以外はv_cruiseへ向けて
    // 加減速する。
    const float decel_dist =
        fabsf(target_speed_mmps_ * target_speed_mmps_ - seg.v_end_mmps * seg.v_end_mmps) /
        (2.0f * params.path_accel);
    float v_next = target_speed_mmps_;
    if (remaining <= decel_dist) {
        if (target_speed_mmps_ > seg.v_end_mmps) {
            v_next = target_speed_mmps_ - params.path_accel * dt_s;
            if (v_next < seg.v_end_mmps) v_next = seg.v_end_mmps;
        } else {
            v_next = target_speed_mmps_ + params.path_accel * dt_s;
            if (v_next > seg.v_end_mmps) v_next = seg.v_end_mmps;
        }
    } else if (target_speed_mmps_ < seg.v_cruise_mmps) {
        v_next = target_speed_mmps_ + params.path_accel * dt_s;
        if (v_next > seg.v_cruise_mmps) v_next = seg.v_cruise_mmps;
    } else if (target_speed_mmps_ > seg.v_cruise_mmps) {
        v_next = target_speed_mmps_ - params.path_accel * dt_s;
        if (v_next < seg.v_cruise_mmps) v_next = seg.v_cruise_mmps;
    }
    target_speed_mmps_ = v_next;

    float next_progress = seg_progress_ + target_speed_mmps_ * dt_s;
    if (next_progress > seg.distance_mm) next_progress = seg.distance_mm;
    seg_progress_ = next_progress;

    target_x_mm_ = seg_start_x_mm_ + seg_progress_ * cosf(seg_start_heading_rad_);
    target_y_mm_ = seg_start_y_mm_ + seg_progress_ * sinf(seg_start_heading_rad_);
    target_heading_rad_ = seg_start_heading_rad_;
    omega_ff_radps_ = 0.0f;

    if (seg_progress_ >= seg.distance_mm) {
        target_speed_mmps_ = seg.v_end_mmps;
        if (seg_index_ + 1 < seg_count_) {
            begin_segment(seg_index_ + 1);
        } else {
            seg_index_ = seg_count_;  // 完了
            target_speed_mmps_ = 0.0f;
        }
    }
}

void PathController::advance_slalom(const Segment& seg, float dt_s) {
    const float omega_mag = fabsf(seg.v_mmps) / seg.radius_mm;
    float next_progress = seg_progress_ + omega_mag * dt_s;
    if (next_progress > seg.angle_rad) next_progress = seg.angle_rad;
    seg_progress_ = next_progress;

    // 半径radius_mm・方向dirの円弧に沿う位置。導出: 進行方向(=向き)に
    // 対して常に半径方向内側に中心があるという拘束から、
    //   x(phi) = x0 + dir*R*(sin(theta0+dir*phi) - sin(theta0))
    //   y(phi) = y0 + dir*R*(cos(theta0) - cos(theta0+dir*phi))
    // (theta0=セグメント開始時の向き、phi=掃引角度[rad]、R=radius_mm)。
    const float theta = seg_start_heading_rad_ + seg.dir * seg_progress_;
    target_x_mm_ = seg_start_x_mm_ +
        seg.dir * seg.radius_mm * (sinf(theta) - sinf(seg_start_heading_rad_));
    target_y_mm_ = seg_start_y_mm_ +
        seg.dir * seg.radius_mm * (cosf(seg_start_heading_rad_) - cosf(theta));
    target_heading_rad_ = theta;
    target_speed_mmps_ = seg.v_mmps;
    omega_ff_radps_ = seg.dir * seg.v_mmps / seg.radius_mm;

    if (seg_progress_ >= seg.angle_rad) {
        if (seg_index_ + 1 < seg_count_) {
            begin_segment(seg_index_ + 1);
        } else {
            seg_index_ = seg_count_;
            target_speed_mmps_ = 0.0f;
            omega_ff_radps_ = 0.0f;
        }
    }
}

void PathController::advance_target(float dt_s) {
    if (seg_index_ >= seg_count_) {
        // 全セグメント完了: ターゲットはその場に留まる(速度ゼロ)。
        target_speed_mmps_ = 0.0f;
        omega_ff_radps_ = 0.0f;
        return;
    }

    // 追従ゲート(2026-08-02追加): ロボットがターゲットから離れすぎたら、
    // 追いつくまでターゲットの前進を凍結する。上のベアリングブレンドで
    // ロボットがターゲット位置へ向き直る復元力が働くので、待っている間に
    // distが縮まりゲートが解除される。dist_to_target_mm_は前tickのupdate()で
    // 計算済みの値(1tick=1msの遅れは無視できる)。
    if (dist_to_target_mm_ > params.path_gate_mm) {
        target_speed_mmps_ = 0.0f;
        omega_ff_radps_ = 0.0f;
        return;
    }

    const Segment& seg = segments_[seg_index_];
    if (seg.type == SegmentType::STRAIGHT) {
        advance_straight(seg, dt_s);
    } else {
        advance_slalom(seg, dt_s);
    }
}

void PathController::update_odometry() {
    const float theta = sensors_.get_angle() - start_heading_rad_;
    const float dist_now = sensors_.get_distance();
    const float delta_mm = dist_now - prev_sensors_dist_mm_;
    prev_sensors_dist_mm_ = dist_now;

    robot_x_mm_ += delta_mm * cosf(theta);
    robot_y_mm_ += delta_mm * sinf(theta);
    robot_theta_rad_ = theta;
}

void PathController::update(float dt_s) {
    advance_target(dt_s);
    update_odometry();

    // 追従(pure pursuit): 仮想ターゲットへのベアリング角と、その方向への
    // 角度誤差(旋回側)、距離誤差(前進側、path_follow_mmを保つ)を計算。
    const float dx = target_x_mm_ - robot_x_mm_;
    const float dy = target_y_mm_ - robot_y_mm_;
    dist_to_target_mm_ = sqrtf(dx * dx + dy * dy);
    // 角度誤差: 追従できているとき(distがpath_follow_mm付近)は「ロボットの
    // 向きとターゲット自身の向き(target_heading、進行方向)の差」を使い高精度に
    // (ベアリング角は追従距離が旋回半径に対して無視できない比率だと幾何学的に
    // ズレるため)。ただしこれだけでは位置の横ずれを戻す復元力が無く、機体の
    // 向きが進行方向から90°を超えて回る(180°/Uターン)と発散する。そこで
    // distが開くほど、ロボット→ターゲット位置への「ベアリング角」の差
    // (=位置復元力)へ滑らかにブレンドする(2026-08-02追加)。
    const float heading_err = normalize_angle(target_heading_rad_ - robot_theta_rad_);
    if (dist_to_target_mm_ > 1.0f && params.path_blend_mm > 0.1f) {
        const float bearing_err = normalize_angle(atan2f(dy, dx) - robot_theta_rad_);
        float blend = (dist_to_target_mm_ - params.path_follow_mm) / params.path_blend_mm;
        if (blend < 0.0f) blend = 0.0f;
        if (blend > 1.0f) blend = 1.0f;
        // heading_err基準にbearing_errへの差分をblendで混ぜる(角度差を
        // normalizeしてから補間することで±πの巻きに強くする)。
        const float delta = normalize_angle(bearing_err - heading_err);
        heading_error_rad_ = normalize_angle(heading_err + blend * delta);
    } else {
        heading_error_rad_ = heading_err;
    }

    const float dist_error_mm = dist_to_target_mm_ - params.path_follow_mm;

    float vbatt = sensors_.get_battery_voltage();
    if (vbatt < 6.0f) vbatt = params.vbatt_nom;  // 起動直後・異常値ガード
    const float kf_fwd = params.kf_duty_per_mps * (params.vbatt_nom / vbatt);

    // 前進側: フィードフォワード(ターゲットの目標速度) + 距離誤差のP
    // (離れすぎたら加速、近すぎたら減速)。
    float duty_common = kf_fwd * (target_speed_mmps_ / 1000.0f) + params.path_kp_fwd * dist_error_mm;
    if (duty_common > params.path_out_max) duty_common = params.path_out_max;
    if (duty_common < -params.path_out_max) duty_common = -params.path_out_max;

    // 旋回側: 幾何フィードフォワード(スラローム中の目標角速度) + 方位
    // 誤差のP + 角速度誤差(ジャイロ実測との差)のD的な補正。
    const float rate_error = omega_ff_radps_ - sensors_.get_gyro_z();
    float duty_diff = params.path_kf_ang * omega_ff_radps_
                     + params.path_kp_ang * heading_error_rad_
                     + params.path_kd_ang * rate_error;
    if (duty_diff > params.path_out_max) duty_diff = params.path_out_max;
    if (duty_diff < -params.path_out_max) duty_diff = -params.path_out_max;

    float u_r = duty_common + duty_diff;
    float u_l = duty_common - duty_diff;
    if (u_r > 1023.0f) u_r = 1023.0f;
    if (u_r < -1023.0f) u_r = -1023.0f;
    if (u_l > 1023.0f) u_l = 1023.0f;
    if (u_l < -1023.0f) u_l = -1023.0f;

    motor_.set_right(static_cast<int16_t>(u_r));
    motor_.set_left(static_cast<int16_t>(u_l));
}
