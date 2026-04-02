/*
 * Hard&Soft Competition - Task 1
 * ESP32-C3 Super Mini — Environmental Monitor
 * Modular architecture — this file is orchestration only.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>
#include <sMQTTBroker.h>
#include <string>

#include "config.h"
#include "sensors/imu.h"
#include "sensors/thermistor.h"
#include "sensors/power.h"
#include "motion/fusion.h"
#include "network/wifi_manager.h"
#include "network/mqtt_client.h"
#include "network/ws_server.h"
#include "ui/display.h"
#include "data/json_builder.h"
#include "data/gpio_monitor.h"
#include "system/cpu_monitor.h"
#include "system/battery.h"
#include "system/rtc_time.h"

// Workaround: ESP32-C3 Arduino needs this
extern "C" int min(int a, int b) { return (a < b) ? a : b; }

// Global objects accessed by modules
Adafruit_SSD1306 g_oledObj(SCREEN_W, SCREEN_H, &Wire, -1);
Adafruit_SSD1306* g_oled = &g_oledObj;
bool g_hasOLED = false;
WiFiClient g_espClient;
PubSubClient g_mqtt(g_espClient);

// Button ISR flags (shared with wifi_manager for skip)
volatile bool isrNextFlag = false;
volatile bool isrPrevFlag = false;
uint32_t gIgnoreButtonsUntilMs = 0;

// ISRs
void IRAM_ATTR isrBtnNext() { isrNextFlag = true; }
void IRAM_ATTR isrBtnPrev() { isrPrevFlag = true; }

// Timing
static uint32_t sLastOtherSensor = 0;
static uint32_t sLastImuPublish = 0;
static uint32_t sLastEnergy = 0;
static uint32_t sLastGyroRead = 0;
static bool sHasGyro = false;

// Button handler
static void handleButtons() {
    uint32_t now = millis();
    if ((int32_t)(gIgnoreButtonsUntilMs - now) > 0) {
        isrNextFlag = false; isrPrevFlag = false; return;
    }
    static uint32_t lastN = 0, lastP = 0;
    if (isrNextFlag) {
        isrNextFlag = false; isrPrevFlag = false;
        if (now - lastN > DEBOUNCE_MS) {
            lastN = now; gIgnoreButtonsUntilMs = now + DEBOUNCE_MS;
            display_set_page((display_get_page() + 1) % NUM_PAGES);
            display_draw_page();
        }
    }
    if (isrPrevFlag) {
        isrPrevFlag = false; isrNextFlag = false;
        if (now - lastP > DEBOUNCE_MS) {
            lastP = now; gIgnoreButtonsUntilMs = now + DEBOUNCE_MS;
            display_set_page((display_get_page() - 1 + NUM_PAGES) % NUM_PAGES);
            display_draw_page();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== Hard&Soft Task 1 — MQTT Monitor ===\n");

    // I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQ);
    Wire.setTimeOut(50);

    // Thermistor ADC
    therm_init();

    // CPU monitor
    cpu_init();
    cpu_start_stress_task();

    // Buttons
    pinMode(PIN_BTN_NEXT, INPUT_PULLUP);
    pinMode(PIN_BTN_PREV, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_NEXT), isrBtnNext, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_PREV), isrBtnPrev, FALLING);

    // IMU interrupt pin
    pinMode(PIN_IMU_INT, INPUT_PULLUP);

    // Display
    display_init(g_oledObj);
    g_hasOLED = display_is_available();
    Serial.printf("[OLED]   %s\n", g_hasOLED ? "OK" : "MISSING");

    // Power (INA219)
    bool hasPower = power_init();
    Serial.printf("[INA219] %s\n", hasPower ? "OK (16V/400mA)" : "MISSING");

    // RTC
    bool hasRTC = rtc_init();
    Serial.printf("[RTC]    %s\n", hasRTC ? "OK" : "MISSING");

    // IMU
    sHasGyro = imu_init();
    Serial.printf("[GYRO]   %s", sHasGyro ? "OK" : "MISSING");
    if (sHasGyro) {
        Serial.printf(" @0x%02X %s\n", imu_get_addr(), imu_is_mpu() ? "(MPU)" : "(LSM)");
        if (imu_is_mpu()) {
            attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imu_isr_data_ready, RISING);
            Serial.printf("[GYRO-INT] Attached GPIO%u\n", PIN_IMU_INT);
        }
        Serial.printf("[MAG]    %s\n", imu_has_mag() ? "AK8963 detected" : "Not present");
    } else {
        Serial.println();
    }

    // WiFi
    isrNextFlag = false; isrPrevFlag = false;
    bool wifiOk = wifi_connect_from_list();
    gIgnoreButtonsUntilMs = millis() + 700;

    if (wifiOk) {
        wifi_sync_ntp();
        rtc_sync_from_system();
        Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WiFi] FAILED — offline mode");
        wifi_start_offline_services();
    }

    // MQTT
    mqtt_init(g_mqtt);
    mqtt_connect();

    // Battery
    battery_load_state();

    // WebSocket
    ws_init();
    Serial.printf("[IMU-WS] ws://%s:%u\n", WiFi.localIP().toString().c_str(), IMU_WS_PORT);

    // GPIO monitor
    gpio_monitor_init();

    sLastOtherSensor = millis();
    sLastImuPublish = millis();
    sLastEnergy = millis();

    delay(1500);
    Serial.println("\n[READY] Loop starting\n");
}

void loop() {
    wifi_update_connectivity();
    ws_loop();

    // Local broker update
    if (wifi_is_local_broker_started()) {
        wifi_get_local_broker().update();
    }

    g_mqtt.loop();

    // MQTT reconnect
    static uint32_t lastRetry = 0;
    if (!g_mqtt.connected() && millis() - lastRetry > 5000) {
        lastRetry = millis();
        mqtt_connect();
    }

    handleButtons();

    // Fast: gyro via interrupt
    if (sHasGyro && mqtt_get_module_gyro()) {
        uint16_t pending = imu_consume_data_ready_count();
        if (pending > 3) pending = 3;
        while (pending--) {
            imu_read();
            sLastGyroRead = millis();
        }
    }

    // Fallback gyro polling
    if (sHasGyro && mqtt_get_module_gyro() && (millis() - sLastGyroRead >= 20)) {
        imu_read();
        sLastGyroRead = millis();
    }

    // IMU WebSocket broadcast
    if (sHasGyro && mqtt_get_module_gyro() && ws_client_count() > 0 &&
        (millis() - sLastImuPublish >= IMU_PUBLISH_INTERVAL_MS)) {
        sLastImuPublish = millis();
        ws_broadcast_imu();
    }

    if (sHasGyro && mqtt_get_module_gyro() && ws_client_count() == 0 &&
        (millis() - sLastImuPublish >= IMU_PUBLISH_IDLE_INTERVAL_MS)) {
        sLastImuPublish = millis();
    }

    // Slow: 1Hz sensors + MQTT + display
    if (millis() - sLastOtherSensor >= 1000) {
        uint32_t now = millis();
        uint32_t dtMs = now - sLastOtherSensor;
        sLastOtherSensor = now;

        rtc_update_timestamp();

        if (mqtt_get_module_current() && power_is_available()) {
            power_read();
        } else if (!mqtt_get_module_current()) {
            // Module off
        }

        if (mqtt_get_module_temp()) {
            therm_read();
        }

        // Battery update
        battery_update(power_get_current_ma(), power_get_voltage(), dtMs, mqtt_get_module_current());
        battery_persist_if_due(now);
        sLastEnergy = now;

        // CPU
        cpu_get_load();

        // GPIO log
        gpio_monitor_update();

        // MQTT publish
        if (g_mqtt.connected()) {
            mqtt_publish_telemetry();
            mqtt_publish_raw();
        } else if (wifi_is_offline_active() && wifi_is_local_broker_started()) {
            char json[1024];
            size_t len = json_build_telemetry(json, sizeof(json));
            if (len < sizeof(json))
                wifi_get_local_broker().publish(std::string(MQTT_TOPIC), std::string(json));

            char raw[1536];
            size_t rawLen = json_build_raw_gpio(raw, sizeof(raw));
            if (rawLen < sizeof(raw))
                wifi_get_local_broker().publish(std::string(MQTT_RAW_TOPIC), std::string(raw));
        }

        display_draw_page();

        Serial.printf("[%s] T=%.1f GX=%.1f V=%.2f I=%.1f CPU=%.1f%%\n",
            rtc_get_timestamp(), therm_read(),
            imu_get_data().gx, power_get_voltage(),
            power_get_current_ma(), cpu_get_load());
    }
}
