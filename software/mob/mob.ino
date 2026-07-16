#include <SPI.h>
#include <WiFi.h>
#include "spi_manager.h"
#include "wall_sensor.h"
#include "battery.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "led.h"

// ============================================================
// グローバルインスタンス
// ============================================================
WallSensor wall_sensor;
Battery    battery;
Encoder    encoder;
IMU        imu(get_imu_spi());
Motor      motor;
Led        led;

// ============================================================
// setup / loop
// ============================================================

void setup() {
    Serial.begin(3000000);
    delay(100);

    // WiFi/Bluetooth 無効化
    WiFi.mode(WIFI_OFF);
    btStop();

    // 各ドライバ初期化
    init_imu_spi();

    wall_sensor.begin();
    battery.begin();
    encoder.begin();
    imu.begin();
    motor.begin();
    led.begin();

    // IMU疎通確認
    uint8_t who = imu.read_who_am_i();
    Serial.printf("#IMU WHO_AM_I = 0x%02X (expect 0x6B)\n", who);

    Serial.printf("#System ready\n");
    Serial.printf("#Commands: MOT,<r>,<l>  WALL,<0|1>  SEN  STOP\n");
}

void loop() {
    static String cmd_buf = "";

    while (Serial.available()) {
        char c = (char)Serial.read();
        led.toggle_rx();
        if (c == '\n') {
            cmd_buf.trim();
            String cmd = cmd_buf;
            cmd_buf = "";

            // ---- コマンド処理 ----

            if (cmd.startsWith("MOT,")) {
                // モーター速度設定: MOT,<right>,<left>  (-1023〜+1023)
                int c1 = cmd.indexOf(',');
                int c2 = cmd.indexOf(',', c1 + 1);
                if (c1 > 0 && c2 > c1) {
                    int16_t r = (int16_t)cmd.substring(c1 + 1, c2).toInt();
                    int16_t l = (int16_t)cmd.substring(c2 + 1).toInt();
                    motor.set_right(r);
                    motor.set_left(l);
                    Serial.printf("#MOT R=%d L=%d\n", r, l);
                } else {
                    Serial.printf("#Invalid MOT format\n");
                }

            } else if (cmd.startsWith("WALL,")) {
                // 壁センサLED有効/無効: WALL,1 or WALL,0
                int c1 = cmd.indexOf(',');
                if (c1 > 0) {
                    bool en = cmd.substring(c1 + 1).toInt() != 0;
                    wall_sensor.set_enabled(en);
                    Serial.printf("#WALL enabled=%d\n", en ? 1 : 0);
                } else {
                    Serial.printf("#Invalid WALL format\n");
                }

            } else if (cmd == "SEN") {
                // センサデータ一括取得
                // 書式: SEN,<gyro_z rad/s>,<batt V>,<wall_r>,<wall_f>,<wall_l>,<enc_r>,<enc_l>
                float gyro_z  = imu.convert_gyro_z_to_radps(imu.read_gyro_z());
                float vbatt   = battery.read_voltage();
                uint16_t wr   = wall_sensor.right();
                uint16_t wf   = wall_sensor.front();
                uint16_t wl   = wall_sensor.left();
                uint16_t enc_r, enc_l;
                encoder.read_both_angles(enc_r, enc_l);
                Serial.printf("SEN,%.4f,%.2f,%u,%u,%u,%u,%u\n",
                              gyro_z, vbatt, wr, wf, wl, enc_r, enc_l);

            } else if (cmd == "STOP") {
                motor.stop();
                Serial.printf("#STOP\n");

            } else if (cmd.length() > 0) {
                motor.stop();
                Serial.printf("#Unknown cmd: %s\n", cmd.c_str());
            }

        } else if (c != '\r') {
            cmd_buf += c;
        }
    }
}
