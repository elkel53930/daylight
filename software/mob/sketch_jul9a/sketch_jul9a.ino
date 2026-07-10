#include "driver/mcpwm_prelude.h"
#include <SPI.h>

// ---- LED ----
#define LED_PIN_R   3
#define LED_PIN_G   20

// ---- Motor driver ----
#define PWM_L_PIN   21
#define CWCCW_L_PIN 47
#define PWM_R_PIN   48
#define CWCCW_R_PIN 45

// ---- Light sensor ----
#define LIGHT_L_EN_PIN  15
#define LIGHT_L_ADC_PIN 16
#define LIGHT_F_EN_PIN   6
#define LIGHT_F_ADC_PIN  7
#define LIGHT_R_EN_PIN   4
#define LIGHT_R_ADC_PIN  5

static int read_light_sensor(int en_pin, int adc_pin) {
    // 1. センサOFF状態でオフセット取得
    int offset = analogRead(adc_pin);
    // 2. センサON
    digitalWrite(en_pin, HIGH);
    // 3. 100us待つ
    delayMicroseconds(100);
    // 4. センサON状態で電圧取得
    int val = analogRead(adc_pin);
    // 5. センサOFF
    digitalWrite(en_pin, LOW);
    // 差分を返す
    return val - offset;
}

// ---- IMU (LSM6DSR) SPI ----
#define IMU_MISO 38
#define IMU_MOSI 39
#define IMU_SCK  40
#define IMU_CS   41

SPIClass imu_spi(HSPI);

static void imu_transfer(uint8_t* rx, const uint8_t* tx, size_t len) {
    imu_spi.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(50);
    for (size_t i = 0; i < len; i++) rx[i] = imu_spi.transfer(tx[i]);
    delayMicroseconds(50);
    digitalWrite(IMU_CS, HIGH);
    imu_spi.endTransaction();
}

static void imu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg, val};
    uint8_t rx[2];
    imu_transfer(rx, tx, 2);
}

static uint8_t imu_read_reg(uint8_t reg) {
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0xFF};
    uint8_t rx[2] = {0, 0};
    imu_transfer(rx, tx, 2);
    return rx[1];
}

static int16_t imu_read_gyro_z() {
    // OUTZ_L_G=0x26, OUTZ_H_G=0x27 を連続読み出し (auto-increment)
    uint8_t tx[3] = {0x26 | 0x80, 0xFF, 0xFF};
    uint8_t rx[3] = {0, 0, 0};
    imu_transfer(rx, tx, 3);
    return (int16_t)((rx[2] << 8) | rx[1]);
}

static bool imu_init() {
    // ソフトウェアリセット
    imu_write_reg(0x12, 0x01);
    delay(50);
    // CTRL3_C: BDU + IF_INC
    imu_write_reg(0x12, 0x44);
    // CTRL2_G: ODR=416Hz, FS=±1000dps
    imu_write_reg(0x11, 0x68);
    // CTRL1_XL: ODR=416Hz, FS=±4g
    imu_write_reg(0x10, 0x68);
    delay(100);
    uint8_t who = imu_read_reg(0x0F);
    Serial.printf("[IMU] WHO_AM_I = 0x%02X (expect 0x6B)\n", who);
    return (who == 0x6B);
}

// MCPWM: 40kHz, 50% duty
// resolution 80MHz / period 2000ticks = 40kHz, compare 1000 = 50%
static void motor_pwm_init() {
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id      = 0;
    timer_cfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = 80000000;
    timer_cfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    timer_cfg.period_ticks  = 2000;  // 80MHz / 2000 = 40kHz
    mcpwm_new_timer(&timer_cfg, &timer);

    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = 0;

    mcpwm_oper_handle_t oper_r = NULL, oper_l = NULL;
    mcpwm_new_operator(&oper_cfg, &oper_r);
    mcpwm_new_operator(&oper_cfg, &oper_l);
    mcpwm_operator_connect_timer(oper_r, timer);
    mcpwm_operator_connect_timer(oper_l, timer);

    mcpwm_comparator_config_t cmp_cfg = {};
    cmp_cfg.flags.update_cmp_on_tez = true;

    mcpwm_cmpr_handle_t cmpr_r = NULL, cmpr_l = NULL;
    mcpwm_new_comparator(oper_r, &cmp_cfg, &cmpr_r);
    mcpwm_new_comparator(oper_l, &cmp_cfg, &cmpr_l);
    mcpwm_comparator_set_compare_value(cmpr_r, 1000);  // 50% duty
    mcpwm_comparator_set_compare_value(cmpr_l, 1000);  // 50% duty

    mcpwm_gen_handle_t gen_r = NULL, gen_l = NULL;
    mcpwm_generator_config_t gen_cfg = {};

    gen_cfg.gen_gpio_num = PWM_R_PIN;
    mcpwm_new_generator(oper_r, &gen_cfg, &gen_r);
    gen_cfg.gen_gpio_num = PWM_L_PIN;
    mcpwm_new_generator(oper_l, &gen_cfg, &gen_l);

    // タイマー周期の先頭でHIGH、コンペアマッチでLOW → Duty=compare/period
    mcpwm_generator_set_action_on_timer_event(gen_r,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_r,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_r, MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_set_action_on_timer_event(gen_l,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_l,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_l, MCPWM_GEN_ACTION_LOW));

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}

void setup() {
    Serial.begin(115200);

    // LED
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    digitalWrite(LED_PIN_R, HIGH);  // 消灯
    digitalWrite(LED_PIN_G, HIGH);  // 消灯

    // 光センサ (左・前・右)
    pinMode(LIGHT_L_EN_PIN, OUTPUT); digitalWrite(LIGHT_L_EN_PIN, LOW);
    pinMode(LIGHT_F_EN_PIN, OUTPUT); digitalWrite(LIGHT_F_EN_PIN, LOW);
    pinMode(LIGHT_R_EN_PIN, OUTPUT); digitalWrite(LIGHT_R_EN_PIN, LOW);
    analogReadResolution(12);  // 12bit (0-4095)

    // モータドライバ方向制御
    pinMode(CWCCW_L_PIN, OUTPUT);
    pinMode(CWCCW_R_PIN, OUTPUT);
    digitalWrite(CWCCW_L_PIN, LOW);
    digitalWrite(CWCCW_R_PIN, LOW);

    // モータドライバ PWM (MCPWM: 40kHz, 50%)
    motor_pwm_init();

    // IMU SPI初期化
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
    imu_spi.begin(IMU_SCK, IMU_MISO, IMU_MOSI, IMU_CS);
    if (!imu_init()) {
        Serial.println("[IMU] init FAILED");
    } else {
        Serial.println("[IMU] init OK");
    }
}

void loop() {
    // Phase A: LED_R点灯 / LED_G消灯、CWCCW = HIGH
    digitalWrite(LED_PIN_R, LOW);   // 点灯
    digitalWrite(LED_PIN_G, HIGH);  // 消灯
    digitalWrite(CWCCW_L_PIN, HIGH);
    digitalWrite(CWCCW_R_PIN, HIGH);
    {
        int16_t raw = imu_read_gyro_z();
        float dps = raw * 0.035f;
        uint32_t t0 = micros();
        int lL = read_light_sensor(LIGHT_L_EN_PIN, LIGHT_L_ADC_PIN);
        int lF = read_light_sensor(LIGHT_F_EN_PIN, LIGHT_F_ADC_PIN);
        int lR = read_light_sensor(LIGHT_R_EN_PIN, LIGHT_R_ADC_PIN);
        uint32_t elapsed = micros() - t0;
        Serial.printf("[Gyro Z] raw=%6d  %.2f deg/s  [Light L/F/R] %4d %4d %4d  (%u us)\n", raw, dps, lL, lF, lR, elapsed);
    }
    delay(500);

    // Phase B: LED_R消灯 / LED_G点灯、CWCCW = LOW
    digitalWrite(LED_PIN_R, HIGH);  // 消灯
    digitalWrite(LED_PIN_G, LOW);   // 点灯
    digitalWrite(CWCCW_L_PIN, LOW);
    digitalWrite(CWCCW_R_PIN, LOW);
    {
        int16_t raw = imu_read_gyro_z();
        float dps = raw * 0.035f;
        uint32_t t0 = micros();
        int lL = read_light_sensor(LIGHT_L_EN_PIN, LIGHT_L_ADC_PIN);
        int lF = read_light_sensor(LIGHT_F_EN_PIN, LIGHT_F_ADC_PIN);
        int lR = read_light_sensor(LIGHT_R_EN_PIN, LIGHT_R_ADC_PIN);
        uint32_t elapsed = micros() - t0;
        Serial.printf("[Gyro Z] raw=%6d  %.2f deg/s  [Light L/F/R] %4d %4d %4d  (%u us)\n", raw, dps, lL, lF, lR, elapsed);
    }
    delay(500);
}
