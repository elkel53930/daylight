#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include <Arduino.h>
#include <stddef.h>
#include "motor.h"
#include "sensors.h"

// 仮想ターゲット追従によるパス追従制御(2026-08-02新規)。
//
// 仮想ターゲットは、ロボットの実際の位置とは独立に、与えられたセグメント
// 列(直進/スラローム旋回)に沿って毎tick進んでいく「ニンジン」
// (pure pursuit の目標点)。ロボットの角度制御は、ロボットから見た
// ターゲットの方向(ベアリング角)ではなく、ロボットの向きとターゲット
// 自身の向き(進行方向)の差を追う(2026-08-02、実機検証により変更)。
// ベアリング角は追従距離(path_follow_mm)がターゲットの旋回半径に対して
// 無視できない比率だと、旋回中は追従距離の分だけ弧の内側/外側にずれた
// 位置からターゲットを見ることになりターゲットの真の進行方向とズレる。
// 距離制御は引き続きターゲットとの直線距離がpath_follow_mm(既定30mm)に
// なるよう速度をP制御(+FF)する。
//
// 座標系は start() を呼んだ瞬間のロボット位置・向きを原点とするローカル
// 平面座標(x=前方、y=左方向)。オドメトリは sensors.get_distance()
// (エンコーダから毎ms更新される平均走行距離)と sensors.get_angle()
// (ジャイロ積分角度)から毎tick積分する(速度の微分推定は行わないため、
// place_controller.cppのような量子化ノイズ対策の窓平均は不要)。
//
// 出力(duty)はHOLD/TURN(place_controller.cpp)と同じ考え方で、前進側
// 補正(duty_common)と旋回側補正(duty_diff)を独立に計算し単純加算する:
//   duty_r = duty_common + duty_diff
//   duty_l = duty_common - duty_diff
// duty_diffのフィードフォワード(path_kf_ang)は、スラローム区間の
// 幾何学的な角速度(dir*v/radius)を使う(pivot_kf/place_controller.cppと
// 同じ考え方: 追従が既に効いている状態からの微修正だけで済むよう、
// 既知の目標角速度を先回りで与える)。
class PathController {
public:
    enum class SegmentType : uint8_t { STRAIGHT, SLALOM };

    // 1個のセグメントを表す。STRAIGHT/SLALOMで使うフィールドが異なる
    // (共用体ではなく、両方のフィールドを持つ単純な構造にしてある。
    // 静的配列で書きやすくするため)。
    struct Segment {
        SegmentType type;

        // STRAIGHT用
        float distance_mm;
        float v_start_mmps;
        float v_cruise_mmps;
        float v_end_mmps;

        // SLALOM用(dir: +1=左/CCW, -1=右/CW)
        float v_mmps;
        float dir;
        float radius_mm;
        float angle_rad;
    };

    PathController(Motor& motor, Sensors& sensors);

    // segments[0..count-1] を順に走行する仮想ターゲットを開始する。
    // segments は呼び出し側が静的な配列として保持し続けること(コピー
    // しない、ポインタを保持するだけ)。
    void start(const Segment* segments, size_t count);

    // 1kHzループから呼ぶ(motion_state == PATH_FOLLOW の間のみ)。
    void update(float dt_s);

    // モーターを止めて終了する。
    void stop();

    bool all_segments_done() const { return seg_index_ >= seg_count_; }

    // テレメトリ用
    float get_target_x_mm() const { return target_x_mm_; }
    float get_target_y_mm() const { return target_y_mm_; }
    float get_robot_x_mm() const { return robot_x_mm_; }
    float get_robot_y_mm() const { return robot_y_mm_; }
    float get_robot_theta_rad() const { return robot_theta_rad_; }
    float get_dist_to_target_mm() const { return dist_to_target_mm_; }
    float get_heading_error_rad() const { return heading_error_rad_; }
    size_t get_seg_index() const { return seg_index_; }

private:
    Motor& motor_;
    Sensors& sensors_;

    const Segment* segments_ = nullptr;
    size_t seg_count_ = 0;
    size_t seg_index_ = 0;

    // オドメトリ基準(start()時点のジャイロ角度・走行距離)
    float start_heading_rad_ = 0.0f;
    float prev_sensors_dist_mm_ = 0.0f;

    // ロボットの現在位置(ローカル座標、mm)・向き(ローカル、rad)
    float robot_x_mm_ = 0.0f;
    float robot_y_mm_ = 0.0f;
    float robot_theta_rad_ = 0.0f;

    // 仮想ターゲットの現在位置・向き・速度
    float target_x_mm_ = 0.0f;
    float target_y_mm_ = 0.0f;
    float target_heading_rad_ = 0.0f;
    float target_speed_mmps_ = 0.0f;

    // 現在セグメント開始時の基準(位置・向き)
    float seg_start_x_mm_ = 0.0f;
    float seg_start_y_mm_ = 0.0f;
    float seg_start_heading_rad_ = 0.0f;
    float seg_progress_ = 0.0f;  // STRAIGHT: 走行距離[mm] / SLALOM: 掃引角度[rad]

    // 旋回幾何フィードフォワード(SLALOM中のみ非ゼロ)
    float omega_ff_radps_ = 0.0f;

    // テレメトリ用
    float dist_to_target_mm_ = 0.0f;
    float heading_error_rad_ = 0.0f;

    void begin_segment(size_t index);
    void advance_target(float dt_s);
    void advance_straight(const Segment& seg, float dt_s);
    void advance_slalom(const Segment& seg, float dt_s);
    void update_odometry();
};

#endif
