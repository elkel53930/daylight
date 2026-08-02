#ifndef PLACE_CONTROLLER_H
#define PLACE_CONTROLLER_H

#include <Arduino.h>
#include "motor.h"
#include "sensors.h"

// 「その場」(=並進せずに同じ位置にとどまる)制御をまとめるクラス。
// start() はその場に静止するだけ、start_turn() はその場旋回(超信地旋回)
// する。両方とも、左右輪速度の和(並進速度成分)をエンコーダFBでゼロへ
// 追い込む制御(2026-08-02実機チューニング済み、HOLDコマンド)は共通で
// 常に動き続ける。start_turn() ではこれに加えて、目標角度(角加速度一定の
// 台形速度プロファイルで滑らかに変化させる)への追従制御を重ね合わせる。
// 並進側の制御が旋回中も動き続けることで、旋回によって機体の位置が
// ずれるのを防ぐ狙い。
//
// 出力(duty)は「並進側の補正(duty_common、左右同じ値)」と「旋回側の
// 補正(duty_diff、左右逆符号)」を独立に計算してから単純加算する:
//   duty_r = duty_common + duty_diff
//   duty_l = duty_common - duty_diff
// (duty_diff > 0 で左/CCW方向に回転する。turn_in_place() 等、旧実装と
// 同じ符号規約)
//
// motion_controller.cpp とは独立の、根本から作り直す新しい制御系列。
class PlaceController {
public:
    PlaceController(Motor& motor, Sensors& sensors);

    // その場静止を開始する(旋回はしない)。
    void start();

    // その場旋回を開始する: 現在角度を基準に delta_angle_rad(正=左/CCW、
    // 単位rad)だけ回転する目標角度プロファイルを生成して追従する。
    // 並進速度ゼロを保つ制御(start()と同じ)も同時に動く。
    void start_turn(float delta_angle_rad);

    // 1kHzループから呼ぶ(motion_state == PLACE_HOLD の間のみ)。
    void update(float dt_s);

    // モーターを止めて終了する。
    void stop();

    bool is_turning() const { return turning_; }

    // テレメトリ用
    float get_vr_mps() const { return vr_filt_mps_; }
    float get_vl_mps() const { return vl_filt_mps_; }
    float get_v_sum_mps() const { return vr_filt_mps_ + vl_filt_mps_; }
    int16_t get_duty() const { return last_duty_; }
    float get_target_angle_rad() const { return turn_target_angle_rad_; }
    float get_omega_target_radps() const { return turn_omega_signed_radps_; }
    int16_t get_duty_diff() const { return last_duty_diff_; }

private:
    // 速度推定を1msごとの生エンコーダ差分から行うと、AS5047の分解能
    // (16384カウント/回転、23.4mm径ホイールで1カウント≈0.0045mm)に対し
    // 1msという時間窓が短すぎ、量子化ノイズ1カウントだけで見かけ上
    // ±4.5mm/s相当の速度ノイズになる(2026-08-02実機、vsumが±15〜20mm/s
    // で振動しゲインを上げても縮まらなかった原因)。速度推定と制御更新の
    // 周期を10ms窓にまとめ、同じ量子化誤差1カウントの影響を約1/10に
    // 圧縮する(制御対象の物理的な時定数に対して10ms=100Hzは十分速い)。
    static constexpr uint8_t VELOCITY_WINDOW_TICKS = 10;
    uint8_t window_tick_count_ = 0;
    float accum_dt_s_ = 0.0f;

    Motor& motor_;
    Sensors& sensors_;

    uint16_t prev_r_angle_ = 0;
    uint16_t prev_l_angle_ = 0;
    bool have_prev_ = false;

    // LPF後の測定車輪速度 [m/s](符号規約はmotion_controller.cppと同じ:
    // 前進方向が正になるよう左輪のみエンコーダ差分を反転して使う)。
    float vr_filt_mps_ = 0.0f;
    float vl_filt_mps_ = 0.0f;

    // 並進速度(vr+vl)をゼロへ追い込むPID
    float integ_ = 0.0f;
    float prev_err_ = 0.0f;

    int16_t last_duty_ = 0;       // 並進側の最終duty(テレメトリ用)
    int16_t last_duty_diff_ = 0;  // 旋回側の最終duty差分(テレメトリ用)

    // その場旋回の状態(start_turn()で初期化)
    bool turning_ = false;
    float turn_dir_ = 1.0f;               // +1: 左(CCW)へ回転中, -1: 右(CW)
    float turn_start_angle_rad_ = 0.0f;   // プロファイル開始時の角度
    float turn_goal_angle_rad_ = 0.0f;    // プロファイルの最終到達角度
    float turn_target_angle_rad_ = 0.0f;  // 現在の目標角度(台形プロファイルで進行)
    float turn_omega_mag_ = 0.0f;         // 現在の目標角速度の大きさ[rad/s]
    float turn_omega_signed_radps_ = 0.0f; // 符号付き目標角速度(テレメトリ用)
    float turn_integ_ = 0.0f;             // 角度誤差の積分項

    static int16_t calc_delta_14bit(uint16_t now, uint16_t prev);
    static float counts_to_m(int16_t delta_counts);

    void reset_common();
    float update_translational(float dt_s);
    float update_turn_profile_and_track(float dt_s);
};

#endif
