#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <atomic>
#include "imu.h"
#include "wall_sensor.h"
#include "battery.h"
#include "encoder.h"

class Sensors {
public:
    Sensors(IMU& imu, WallSensor& wall_sensor, Battery& battery, Encoder& encoder);
    
    // Core0から呼び出される関数: センサー値を読み取ってatomic変数に格納
    void update(uint32_t time_delta_ms);
    
    // Core1のloop()から呼び出される関数: 格納された値を読み出す
    float get_gyro_z() const;           // Z軸ジャイロ角速度（rad/s）
    float get_accel_forward() const;    // 前後方向加速度（m/s^2、+=前進側、IMU Y軸。2026-08-02実機確認）
    uint16_t get_lf() const;            // 左前壁センサー値（D では前壁F を返す）
    uint16_t get_rf() const;            // 右前壁センサー値（D では前壁F を返す）
    uint16_t get_ls() const;            // 左側壁センサー値
    uint16_t get_rs() const;            // 右側壁センサー値
    float get_battery_voltage() const;  // バッテリー電圧
    uint16_t get_right_wheel_angle() const;  // 右車輪角度（生値: 0-16383）
    uint16_t get_left_wheel_angle() const;   // 左車輪角度（生値: 0-16383）
    
    // オドメトリ関連
    float get_distance() const;              // 移動距離（mm）
    float get_angle() const;                 // 姿勢角度（rad）
    void reset_distance();                   // 距離リセット
    void reset_angle();                      // 角度リセット(0にする)
    void set_angle(float rad);               // 角度を任意の値に上書き(カメラ補正等、外部の絶対基準での補正用)
    void set_stationary(bool s);             // 停止中フラグ(true=モーター停止中)。停止中はジャイロ角度積分を凍結する

    // ジャイロキャリブレーション
    void calibrate_gyro();                   // ジャイロオフセットを計算（100回 サンプル平均）
    float get_gyro_offset() const;           // 現在のジャイロオフセット値を取得
    bool is_calibrating() const;             // キャリブレーション中かどうか
    
private:
    // センサーへの参照
    IMU& imu_;
    WallSensor& wall_sensor_;
    Battery& battery_;
    Encoder& encoder_;
    
    // Atomic変数でセンサーデータを保持
    std::atomic<float> gyro_z_;         // Z軸ジャイロ角速度（rad/s）
    std::atomic<float> accel_forward_;  // 前後方向加速度（m/s^2、IMU Y軸）
    std::atomic<uint16_t> f_;           // 前壁センサー値（D: front()）
    std::atomic<uint16_t> ls_;          // 左側壁センサー値
    std::atomic<uint16_t> rs_;          // 右側壁センサー値
    std::atomic<float> battery_voltage_; // バッテリー電圧
    std::atomic<uint16_t> right_wheel_angle_; // 右車輪角度（生値: 0-16383）
    std::atomic<uint16_t> left_wheel_angle_;  // 左車輪角度（生値: 0-16383）
    
    // オドメトリ用変数
    std::atomic<float> distance_;       // 累積移動距離（mm）
    std::atomic<float> angle_;          // 累積姿勢角度（rad）
    uint16_t prev_right_angle_;         // 前回の右エンコーダ値
    uint16_t prev_left_angle_;          // 前回の左エンコーダ値
    
    // ジャイロキャリブレーション用
    std::atomic<float> gyro_offset_;    // ジャイロオフセット（rad/s）
    std::atomic<bool> calibrating_;     // キャリブレーション中フラグ
    std::atomic<bool> stationary_;      // モーター停止中フラグ(true=角度積分を凍結)
    int calib_count_;                   // キャリブレーションサンプルカウント
    float calib_sum_;                   // キャリブレーションサンプル合計
    uint32_t calib_interval_ms_;        // キャリブレーションサンプル間隔
    uint32_t calib_timer_ms_;           // キャリブレーションタイマー
    
    // ロボット物理パラメータ
    static constexpr float WHEEL_DIAMETER = 23.0f;     // ホイール直径（mm）
    static constexpr float GEAR_RATIO = 1.0f;          // エンコーダ:ホイールの ギア比(直結)
    static constexpr float ENCODER_RESOLUTION = 16384.0f; // エンコーダ分解能（14bit）
    static constexpr float WHEEL_BASE = 76.0f;         // 車輪間距離（mm）
 
    // エンコーダカウントから移動距離への変換係数（mm/count）
    static constexpr float COUNT_TO_MM = (WHEEL_DIAMETER * 3.14159265359f) / (ENCODER_RESOLUTION * GEAR_RATIO);
};

#endif
