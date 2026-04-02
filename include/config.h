#pragma once

#include <cstddef>
#include <cstdint>

// ================================================================
// PIN ASSIGNMENT
// ================================================================
static const uint8_t PIN_SDA       = 8;
static const uint8_t PIN_SCL       = 9;
static const uint8_t PIN_BTN_NEXT  = 3;
static const uint8_t PIN_BTN_PREV  = 4;
static const uint8_t PIN_THERM     = 0;
static const uint8_t PIN_IMU_INT   = 10;

// GPIO snapshot pins (avoid GPIO18/19 = USB D-/D+ on ESP32-C3)
static const uint8_t GPIO_PUBLISH_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
static const size_t GPIO_PUBLISH_PIN_COUNT = sizeof(GPIO_PUBLISH_PINS) / sizeof(GPIO_PUBLISH_PINS[0]);

// ================================================================
// I2C
// ================================================================
static const uint32_t I2C_FREQ       = 100000;  // 100kHz for DS1307
static const uint8_t I2C_ADDR_OLED   = 0x3C;
static const uint8_t I2C_ADDR_INA219 = 0x40;
static const uint8_t I2C_ADDR_RTC    = 0x68;
static const uint8_t I2C_ADDR_RTC_ALT = 0x60;

// ================================================================
// IMU ADDRESSES
// ================================================================
static const uint8_t IMU_ADDR_LSM6   = 0x6A;
static const uint8_t IMU_ADDR_MPU0   = 0x68;
static const uint8_t IMU_ADDR_MPU1   = 0x69;

// ================================================================
// WIFI & MQTT
// ================================================================
static const char* WIFI_SSIDS[]      = {"stanislav", "Network", "DIGI", "DIGI-75hC"};
static const char* WIFI_PASSWORDS[]  = {"stas1524", "19911313", "27mc2004ca", "9T2Du96euz"};
static const size_t WIFI_NETWORK_COUNT = 4;
static const uint8_t WIFI_ATTEMPT_TIMEOUT_SEC = 12;
static const char* AP_SSID           = "HS-ESP32-OFF";
static const char* AP_PASS           = "esp32hs123";
static const uint8_t AP_CHANNEL      = 6;
static const uint8_t AP_MAX_CONN     = 4;

static const char* MQTT_BROKER       = "broker.emqx.io";
static const uint16_t MQTT_PORT      = 1883;
static const char* MQTT_TOPIC        = "hardandsoft/esp32/data";
static const char* MQTT_RAW_TOPIC    = "hardandsoft/esp32/gpio_raw";
static const char* MQTT_CMD_TOPIC    = "hardandsoft/esp32/cmd";
static const char* MQTT_STATE_TOPIC  = "hardandsoft/esp32/state";
static const char* MQTT_USER         = "emqx";
static const char* MQTT_PASS_MQTT    = "public";

// ================================================================
// TIMING
// ================================================================
static const uint32_t IMU_PUBLISH_INTERVAL_MS      = 16;   // ~60Hz
static const uint32_t IMU_PUBLISH_IDLE_INTERVAL_MS = 50;
static const uint16_t IMU_WS_PORT                  = 8001;

// ================================================================
// DISPLAY
// ================================================================
static const uint8_t SCREEN_W        = 128;
static const uint8_t SCREEN_H        = 64;
static const uint8_t NUM_PAGES       = 10;
static const uint16_t DEBOUNCE_MS    = 200;
static const uint8_t OLED_TITLE_H    = 16;
static const uint8_t OLED_TITLE_TEXT_Y = 4;

// ================================================================
// THERMISTOR (NTC 10k divider)
// ================================================================
static const float ADC_MAX_F         = 4095.0f;
static const float R_FIXED           = 10000.0f;
static const float R_NOMINAL         = 10000.0f;
static const float T_NOMINAL         = 25.0f;
static const float B_COEFF           = 3950.0f;
static const float THERM_ADC_REF_V   = 3.3f;
static const float THERM_SUPPLY_FALLBACK_V = 4.0f;
static const float THERM_FILTER_ALPHA = 0.35f;
static const bool  THERM_NTC_TO_GND  = true;
static const float THERM_CAL_GAIN    = 1.0f;
static const float THERM_CAL_OFFSET_C = -2.0f;

// ================================================================
// IMU REGISTERS & SCALES
// ================================================================
static const uint8_t LSM_WHO_AM_I_REG  = 0x0F;
static const uint8_t LSM_WHO_AM_I_VAL  = 0x69;
static const uint8_t LSM_CTRL2_G       = 0x11;
static const uint8_t LSM_OUTX_L_G      = 0x22;

static const uint8_t MPU_WHO_AM_I_REG  = 0x75;
static const uint8_t MPU_WHO_AM_I_6500 = 0x70;
static const uint8_t MPU_WHO_AM_I_9250 = 0x71;
static const uint8_t MPU_SMPLRT_DIV    = 0x19;
static const uint8_t MPU_CONFIG_REG    = 0x1A;
static const uint8_t MPU_PWR_MGMT_1    = 0x6B;
static const uint8_t MPU_PWR_MGMT_2    = 0x6C;
static const uint8_t MPU_GYRO_CONFIG   = 0x1B;
static const uint8_t MPU_ACCEL_CONFIG  = 0x1C;
static const uint8_t MPU_ACCEL_XOUT_H  = 0x3B;
static const uint8_t MPU_GYRO_XOUT_H   = 0x43;
static const uint8_t MPU_INT_PIN_CFG   = 0x37;
static const uint8_t MPU_INT_ENABLE    = 0x38;
static const uint8_t MPU_INT_STATUS    = 0x3A;

static const uint8_t AK8963_ADDR       = 0x0C;
static const uint8_t AK8963_WHO_AM_I   = 0x00;
static const uint8_t AK8963_ST1        = 0x02;
static const uint8_t AK8963_XOUT_L     = 0x03;
static const uint8_t AK8963_CNTL1      = 0x0A;
static const uint8_t AK8963_ASAX       = 0x10;
static const uint8_t AK8963_WHO_AM_I_VAL = 0x48;

static const float MPU_GYRO_SCALE_DPS  = 1.0f / 65.5f;   // +-500 dps
static const float LSM_GYRO_SCALE_DPS  = 0.07f;           // +-2000 dps
static const float MPU_ACCEL_SCALE_G   = 1.0f / 16384.0f; // +-2g
static const uint8_t MPU_BIAS_SAMPLES  = 96;

static const float GYRO_FILTER_ALPHA   = 0.25f;
static const float GYRO_DEADBAND_DPS   = 0.5f;
static const float GYRO_STILL_DPS      = 2.5f;
static const uint8_t GYRO_STILL_REQUIRED_SAMPLES = 6;

// ================================================================
// FUSION / MOTION
// ================================================================
static const uint8_t ZUPT_STILL_REQUIRED_SAMPLES = 10;
static const uint32_t BIAS_RECAL_INTERVAL_MS     = 300000;
static const float GYRO_BIAS_STILL_ALPHA         = 0.015f;
static const float GYRO_BIAS_RECAL_ALPHA         = 0.06f;
static const float ACCEL_BIAS_STILL_ALPHA        = 0.01f;
static const float ACCEL_BIAS_RECAL_ALPHA        = 0.04f;
static const float ZUPT_VEL_DECAY                = 0.02f;
static const float ZUPT_POS_HOLD_ALPHA           = 0.002f;
static const float STATIONARY_WORLD_ACC_G        = 0.08f;

static const float KALMAN_Q_ANGLE    = 0.001f;
static const float KALMAN_Q_BIAS     = 0.003f;
static const float KALMAN_R_MEASURE  = 0.03f;
static const float YAW_MAG_BLEND_MIN = 0.03f;
static const float YAW_MAG_BLEND_MAX = 0.14f;
static const float QUAT_ACC_GAIN     = 1.8f;
static const float QUAT_MAG_GAIN     = 1.4f;

// ================================================================
// BATTERY
// ================================================================
static const float BATTERY_CAPACITY           = 3200.0f;
static const float BATT_REST_CURRENT_MA       = 25.0f;
static const uint32_t BATT_REST_TIME_MS       = 30000;
static const float BATT_OCV_MISMATCH_RECOVER_PCT = 20.0f;
static const uint32_t BATT_SAVE_INTERVAL_MS   = 60000;
static const float BATT_PRESENT_ON_V          = 2.8f;
static const float BATT_PRESENT_OFF_V         = 2.4f;
static const uint32_t BATT_PRESENCE_DEBOUNCE_MS = 5000;
static const float BATT_CURRENT_DEADBAND_MA   = 5.0f;
static const char* PREF_NS                    = "battery";
static const char* PREF_KEY_USED_MAH          = "used_mAh";

// ================================================================
// OLED LOG
// ================================================================
static const uint8_t OLED_LOG_LINES    = 5;
static const uint8_t OLED_LOG_LINE_LEN = 21;
static const uint8_t I2C_SCAN_MAX_DEVICES = 16;
