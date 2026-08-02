#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "motor.h"
#include "sensors.h"

// 低レベルの車輪速度PID(MOT/DUTYコマンド用)。距離・角度プロファイルを
// 持った上位の移動制御(旧FWD/STOP/TURN等)は2026-08-02に削除し、根本から
// 作り直し中(place_controller.cpp等)。このクラスはMOT(左右輪に同じ目標
// 速度を即座に設定)とDUTY(速度PID非経由の生duty直接指令)のためだけに
// 残している。
class MotionController {
public:
    MotionController(Motor& motor, Sensors& sensors);

    // Call at 1kHz (or as frequently as Control0 loop) to update control output.
    void update(uint32_t dt_ms);

    // 左右輪に同じ目標速度を設定する(MOTコマンド用)。
    void forward(float speed_mps);

    // 校正・診断用: 左右独立にduty(-1023〜+1023)を直接指令する。速度PID
    // (pid_r_/pid_l_)は経由しない。速度制御に戻ったときに古い積分値で
    // 暴れないよう積分器はリセットする。
    void set_duty_direct(int16_t r_duty, int16_t l_duty);

    // Stop everything.
    void stop();

    // テレメトリ用: LPF後の測定車輪速度と最終duty出力
    float get_vr_filt_mps() const { return vr_filt_mps_; }
    float get_vl_filt_mps() const { return vl_filt_mps_; }
    int16_t get_duty_r() const { return last_duty_r_; }
    int16_t get_duty_l() const { return last_duty_l_; }

private:
    enum class Mode {
        STOP,
        FORWARD,
        DUTY_DIRECT  // 校正・診断用: 速度PIDを経由せず直接duty指令(set_duty_direct)
    };

    Motor& motor_;
    Sensors& sensors_;

    Mode mode_ = Mode::STOP;

    // Targets
    float vr_ref_mps_ = 0.0f;
    float vl_ref_mps_ = 0.0f;

    // Internal per-wheel speed PID
    struct SpeedPID {
        float kp;
        float ki;
        float kd;
        float integ;
        float prev_err;
        float out_min;
        float out_max;
        float step(float err, float dt_s);
        void reset();
    };

    SpeedPID pid_r_;
    SpeedPID pid_l_;

    // Encoder tracking for velocity estimation
    uint16_t prev_r_angle_ = 0;
    uint16_t prev_l_angle_ = 0;
    bool have_prev_ = false;

    // LPF後の測定速度 [m/s]
    float vr_filt_mps_ = 0.0f;
    float vl_filt_mps_ = 0.0f;

    // 最終duty出力(テレメトリ用)
    int16_t last_duty_r_ = 0;
    int16_t last_duty_l_ = 0;

    // Helpers
    static int16_t calc_delta_14bit(uint16_t now, uint16_t prev);
    static float counts_to_m(int16_t delta_counts);

    void set_targets_mps(float vr, float vl);
    void apply_speed_pid(uint32_t dt_ms);
};

#endif
