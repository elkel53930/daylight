#include "sensors.h"

Sensors::Sensors(IMU& imu, WallSensor& wall_sensor, Battery& battery, Encoder& encoder)
    : imu_(imu), wall_sensor_(wall_sensor), battery_(battery), encoder_(encoder),
      gyro_z_(0.0f), accel_forward_(0.0f), f_(0), ls_(0), rs_(0),
      battery_voltage_(0.0f), right_wheel_angle_(0), left_wheel_angle_(0),
      distance_(0.0f), angle_(0.0f), prev_right_angle_(0), prev_left_angle_(0),
      gyro_offset_(0.0f), calibrating_(false), calib_count_(0), calib_sum_(0.0f),
      calib_interval_ms_(5), calib_timer_ms_(0) {
}

void Sensors::update(uint32_t time_delta_ms) {
    // IMUデータの読み取り
    int16_t raw_gyro_z = imu_.read_gyro_z();
    float gyro_z_radps = imu_.convert_gyro_z_to_radps(raw_gyro_z);
    gyro_z_.store(gyro_z_radps, std::memory_order_relaxed);

    int16_t raw_accel_y = imu_.read_accel_y();
    float accel_forward_mps2 = imu_.convert_accel_to_mps2(raw_accel_y);
    accel_forward_.store(accel_forward_mps2, std::memory_order_relaxed);

    // キャリブレーション処理（非ブロッキング）
    if (calibrating_.load(std::memory_order_relaxed)) {
        calib_timer_ms_ += time_delta_ms;
        if (calib_timer_ms_ >= calib_interval_ms_) {
            calib_timer_ms_ = 0;
            calib_sum_ += gyro_z_radps;
            calib_count_++;
            
            if (calib_count_ >= 100) {
                // 100サンプル完了
                float offset = calib_sum_ / 100.0f;
                gyro_offset_.store(offset, std::memory_order_relaxed);
                calibrating_.store(false, std::memory_order_relaxed);
            }
        }
    }
    
    // 壁センサー値の読み取り（D: front/left/right）
    uint16_t f_value = wall_sensor_.front();
    uint16_t ls_value = wall_sensor_.left();
    uint16_t rs_value = wall_sensor_.right();
    f_.store(f_value, std::memory_order_relaxed);
    ls_.store(ls_value, std::memory_order_relaxed);
    rs_.store(rs_value, std::memory_order_relaxed);
    
    // バッテリー電圧の読み取り
    float voltage = battery_.read_voltage();
    battery_voltage_.store(voltage, std::memory_order_relaxed);
    
    // エンコーダー角度の読み取り（生値）
    uint16_t r_angle = encoder_.read_right_angle();
    uint16_t l_angle = encoder_.read_left_angle();
    right_wheel_angle_.store(r_angle, std::memory_order_relaxed);
    left_wheel_angle_.store(l_angle, std::memory_order_relaxed);
    
    // オドメトリ計算
    // エンコーダの差分計算（オーバーフロー対応: 14bit 0..16383）
    auto calc_delta_14bit = [](uint16_t now, uint16_t prev) -> int16_t {
        int32_t d = static_cast<int32_t>(now) - static_cast<int32_t>(prev);
        d = (d % 16384 + 16384) % 16384;
        if (d > 8192) d -= 16384;
        return static_cast<int16_t>(d);
    };

    int16_t delta_r = calc_delta_14bit(r_angle, prev_right_angle_);
    int16_t delta_l = calc_delta_14bit(l_angle, prev_left_angle_);
    
    // 移動距離計算（mm）
    // 前進方向の反転(2026-07-18)に合わせて符号を反転。motion_controller.cpp の
    // 速度PID計算と揃えること。
    float distance_r = delta_r * COUNT_TO_MM;
    float distance_l = -1 * delta_l * COUNT_TO_MM;
    float delta_distance = (distance_r + distance_l) / 2.0f;
    
    // 角度計算（rad）
    float delta_angle_gyro = gyro_z_radps * (static_cast<float>(time_delta_ms) / 1000.0f);
    float delta_angle_encoder = (distance_r - distance_l) / WHEEL_BASE;
    
    // ジャイロ積分を優先（相補フィルタは後で追加可）
    float delta_angle = delta_angle_gyro;
    
    // 累積値更新
    distance_.store(distance_.load(std::memory_order_relaxed) + delta_distance, std::memory_order_relaxed);
    angle_.store(angle_.load(std::memory_order_relaxed) + delta_angle, std::memory_order_relaxed);
    
    // 前回値更新
    prev_right_angle_ = r_angle;
    prev_left_angle_ = l_angle;
}

float Sensors::get_gyro_z() const {
    float raw_gyro = gyro_z_.load(std::memory_order_relaxed);
    float offset = gyro_offset_.load(std::memory_order_relaxed);
    return raw_gyro - offset;
}

float Sensors::get_accel_forward() const {
    return accel_forward_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_lf() const {
    // D: 前壁F を返す（T互換）
    return f_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_rf() const {
    // D: 前壁F を返す（T互換）
    return f_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_ls() const {
    return ls_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_rs() const {
    return rs_.load(std::memory_order_relaxed);
}

float Sensors::get_battery_voltage() const {
    return battery_voltage_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_right_wheel_angle() const {
    return right_wheel_angle_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_left_wheel_angle() const {
    return left_wheel_angle_.load(std::memory_order_relaxed);
}

float Sensors::get_distance() const {
    return distance_.load(std::memory_order_relaxed);
}

float Sensors::get_angle() const {
    return angle_.load(std::memory_order_relaxed);
}

void Sensors::reset_distance() {
    distance_.store(0.0f, std::memory_order_relaxed);
}

void Sensors::reset_angle() {
    angle_.store(0.0f, std::memory_order_relaxed);
}

void Sensors::set_angle(float rad) {
    angle_.store(rad, std::memory_order_relaxed);
}

void Sensors::calibrate_gyro() {
    calibrating_.store(true, std::memory_order_relaxed);
    calib_count_ = 0;
    calib_sum_ = 0.0f;
}

float Sensors::get_gyro_offset() const {
    return gyro_offset_.load(std::memory_order_relaxed);
}

bool Sensors::is_calibrating() const {
    return calibrating_.load(std::memory_order_relaxed);
}
