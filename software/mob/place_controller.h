#ifndef PLACE_CONTROLLER_H
#define PLACE_CONTROLLER_H

#include <Arduino.h>
#include "motor.h"
#include "sensors.h"

// その場旋回の作り込みの第一歩として、まず「その場に静止する」ことだけを
// 目的とした制御器。左右輪の速度の和(=並進速度成分)をエンコーダで測定し、
// これがゼロになるよう左右対称(同じduty)の補正をかけ続けるクローズド
// ループ制御。左右輪の速度差(=回転成分)には触れない。
//
// motion_controller.cpp とは独立の、根本から作り直す新しい制御系列の
// 最初のモジュール(motor/encoder/sensors等の低レベル層はそのまま流用)。
// 将来、回転成分側にも目標値(角速度)を持たせられるように拡張し、
// 「並進ゼロを保ったままその場旋回する」制御へ発展させる想定。
class PlaceController {
public:
    PlaceController(Motor& motor, Sensors& sensors);

    // 開始時に呼ぶ: PID積分項・速度推定の前回値をリセットする。
    void start();

    // 1kHzループから呼ぶ(motion_state == PLACE_HOLD の間のみ)。
    void update(float dt_s);

    // モーターを止めて終了する。
    void stop();

    // テレメトリ用
    float get_vr_mps() const { return vr_filt_mps_; }
    float get_vl_mps() const { return vl_filt_mps_; }
    float get_v_sum_mps() const { return vr_filt_mps_ + vl_filt_mps_; }
    int16_t get_duty() const { return last_duty_; }

private:
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

    int16_t last_duty_ = 0;

    static int16_t calc_delta_14bit(uint16_t now, uint16_t prev);
    static float counts_to_m(int16_t delta_counts);
};

#endif
