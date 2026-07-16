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
    void backward(float speed_mps);

    // Start/continue a turn in place. Positive angle: turn left (CCW) by convention.
    // Completion is judged by the caller (absolute angle vs. tolerance); this
    // only sets the per-wheel speed targets for the requested direction.
    void turn_in_place(float speed_mps, float target_angle_rad);

    // Stop everything.
    void stop();

private:
    enum class Mode {
        STOP,
        FORWARD,
        BACKWARD,
        TURN
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

    // Helpers
    static int16_t calc_delta_14bit(uint16_t now, uint16_t prev);
    static float counts_to_m(int16_t delta_counts);

    void set_targets_mps(float vr, float vl);
    void apply_speed_pid(uint32_t dt_ms);
};

#endif
