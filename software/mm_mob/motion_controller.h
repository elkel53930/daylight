#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "motor.h"
#include "sensors.h"

// High-level motion control wrapper.
// D用に、Motor インターフェンスを MCPWM（−1023〜+1023）に適合させている。
// forward(): drive forward at a target speed while accepting a lateral error value (from wall sensors, etc.).
// backward(): drive backward at a target speed.
// turn_in_place(): turn in place by running wheels in opposite directions.
class MotionController {
public:
    MotionController(Motor& motor, Sensors& sensors);

    // Call at 1kHz (or as frequently as Control0 loop) to update control output.
    void update(uint32_t dt_ms);

    // Start/continue forward motion.
    // lateral_error: left/right deviation (unitless for now). Positive means drift to right (example).
    void forward(float speed_mps, float lateral_error);

    // Start/continue backward motion.
    // lateral_error: same convention as forward()(既定0.0で従来と同じ動作)。
    void backward(float speed_mps, float lateral_error = 0.0f);

    // Start/continue a turn in place. Positive angle: turn left (CCW) by convention.
    // Completion is judged by the caller (absolute angle vs. tolerance); this
    // only sets the per-wheel speed targets for the requested direction.
    void turn_in_place(float speed_mps, float target_angle_rad);

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
        BACKWARD,
        TURN,
        DUTY_DIRECT  // 校正・診断用: 速度PIDを経由せず直接duty指令(set_duty_direct)
    };

    Motor& motor_;
    Sensors& sensors_;

    Mode mode_ = Mode::STOP;

    // Targets
    float vr_ref_mps_ = 0.0f;
    float vl_ref_mps_ = 0.0f;

    // Lateral correction gain (rad/s per unit lateral error)
    float k_lateral_ = 1.0f;

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

    // 旋回中の左右速度同期(積分項)。TURN以外では0にリセットする。
    float turn_sync_integ_ = 0.0f;

    // 旋回中の左右速度同期誤差を追加LPFした値(個別車輪PIDと同じ生の
    // vr_filt_mps_/vl_filt_mps_ に反応すると結合振動するため、同期
    // ループ専用にさらに緩いLPFをかけて低周波成分だけに反応させる)。
    float turn_sync_err_filt_ = 0.0f;

    // Helpers
    static int16_t calc_delta_14bit(uint16_t now, uint16_t prev);
    static float counts_to_m(int16_t delta_counts);

    void set_targets_mps(float vr, float vl);
    void apply_speed_pid(uint32_t dt_ms);
};

#endif
