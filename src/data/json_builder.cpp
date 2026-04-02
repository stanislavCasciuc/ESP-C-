#include "json_builder.h"
#include "config.h"
#include "../sensors/imu.h"
#include "../sensors/thermistor.h"
#include "../sensors/power.h"
#include "../motion/fusion.h"
#include "../network/mqtt_client.h"
#include "../system/cpu_monitor.h"
#include "../system/battery.h"
#include "../system/rtc_time.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <cmath>

size_t json_build_telemetry(char* buf, size_t bufSize) {
    bool modTemp = mqtt_get_module_temp();
    bool modGyro = mqtt_get_module_gyro();
    bool modCurrent = mqtt_get_module_current();
    bool hasGyro_ = imu_get_addr() != 0;
    const ImuData& imu = imu_get_data();
    const FusionState& fs = fusion_get_state();

    JsonDocument doc;
    doc["ts"] = rtc_get_timestamp();
    doc["up"] = rtc_get_uptime();

    if (modTemp && therm_is_valid()) {
        float t = roundf(therm_read() * 10.0f) / 10.0f;
        // Note: calling therm_read() again here is fine, it returns cached filtered value
        // Actually let's just read from what we have
        doc["temp"] = t; doc["temperature"] = t; doc["ntc_c"] = t;
    } else {
        doc["temp"] = nullptr; doc["temperature"] = nullptr; doc["ntc_c"] = nullptr;
    }
    doc["temp_raw"] = (modTemp && therm_is_valid()) ? therm_get_raw() : 0;
    doc["ntc_raw"]  = (modTemp && therm_is_valid()) ? therm_get_raw() : 0;

    if (modGyro && hasGyro_) {
        doc["gx"] = roundf(imu.gx * 10) / 10.0f;
        doc["gy"] = roundf(imu.gy * 10) / 10.0f;
        doc["gz"] = roundf(imu.gz * 10) / 10.0f;
        doc["gabs"] = roundf(imu.gyroAbs * 10) / 10.0f;
        doc["ax"] = roundf(imu.ax * 1000) / 1000.0f;
        doc["ay"] = roundf(imu.ay * 1000) / 1000.0f;
        doc["az"] = roundf(imu.az * 1000) / 1000.0f;
        doc["roll"]    = roundf(fs.roll * 100) / 100.0f;
        doc["pitch"]   = roundf(fs.pitch * 100) / 100.0f;
        doc["yaw"]     = roundf(fs.yaw * 100) / 100.0f;
        doc["heading"] = roundf(fs.heading * 100) / 100.0f;
        doc["q0"] = roundf(fs.q0 * 10000) / 10000.0f;
        doc["q1"] = roundf(fs.q1 * 10000) / 10000.0f;
        doc["q2"] = roundf(fs.q2 * 10000) / 10000.0f;
        doc["q3"] = roundf(fs.q3 * 10000) / 10000.0f;
        if (imu.mag_present) {
            doc["mx"] = roundf(imu.mx * 10) / 10.0f;
            doc["my"] = roundf(imu.my * 10) / 10.0f;
            doc["mz"] = roundf(imu.mz * 10) / 10.0f;
        } else { doc["mx"]=nullptr; doc["my"]=nullptr; doc["mz"]=nullptr; }
        doc["mag_present"] = imu.mag_present;
        doc["imu_addr"] = imu_get_addr();
    } else {
        const char* nullKeys[] = {"gx","gy","gz","gabs","ax","ay","az","roll","pitch","yaw","heading",
                                  "q0","q1","q2","q3","mx","my","mz","imu_addr"};
        for (auto k : nullKeys) doc[k] = nullptr;
        doc["mag_present"] = false;
    }

    doc["rssi"] = (int16_t)WiFi.RSSI();
    doc["cpu"]  = (int)roundf(cpu_get_load());

    if (modCurrent) {
        doc["v"]  = roundf(power_get_voltage() * 100) / 100.0f;
        doc["ma"] = roundf(power_get_current_ma() * 10) / 10.0f;
        doc["mw"] = (int)roundf(power_get_power_mw());
        doc["mah"]  = roundf(battery_get_used_mah() * 100) / 100.0f;
        doc["batt"] = roundf(battery_get_percent() * 10) / 10.0f;
    } else {
        doc["v"]=nullptr; doc["ma"]=nullptr; doc["mw"]=nullptr;
        doc["mah"]=nullptr; doc["batt"]=nullptr;
    }

    doc["batt_min"] = (modCurrent && battery_is_present() && battery_get_life_min() > 0)
                      ? (int)battery_get_life_min() : -1;
    doc["ssid"] = WiFi.SSID();
    doc["ip"] = WiFi.localIP().toString();
    doc["mac"] = WiFi.macAddress();
    doc["channel"] = (int)WiFi.channel();

    return serializeJson(doc, buf, bufSize);
}

size_t json_build_imu(char* buf, size_t bufSize) {
    const FusionState& fs = fusion_get_state();
    const ImuData& imu = imu_get_data();

    float q0 = isfinite(fs.q0)?fs.q0:1, q1 = isfinite(fs.q1)?fs.q1:0;
    float q2 = isfinite(fs.q2)?fs.q2:0, q3 = isfinite(fs.q3)?fs.q3:0;
    float qn = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    if (qn > 1e-6f) { q0/=qn; q1/=qn; q2/=qn; q3/=qn; }
    else { q0=1; q1=q2=q3=0; }

    JsonDocument doc;
    doc["up"] = rtc_get_uptime();
    doc["q0"] = roundf(q0 * 10000) / 10000.0f;
    doc["q1"] = roundf(q1 * 10000) / 10000.0f;
    doc["q2"] = roundf(q2 * 10000) / 10000.0f;
    doc["q3"] = roundf(q3 * 10000) / 10000.0f;
    doc["roll"]  = roundf(fs.roll * 100) / 100.0f;
    doc["pitch"] = roundf(fs.pitch * 100) / 100.0f;
    doc["yaw"]   = roundf(fs.yaw * 100) / 100.0f;
    doc["gx"] = roundf(imu.gx * 10) / 10.0f;
    doc["gy"] = roundf(imu.gy * 10) / 10.0f;
    doc["gz"] = roundf(imu.gz * 10) / 10.0f;
    doc["ax"] = roundf(imu.ax * 1000) / 1000.0f;
    doc["ay"] = roundf(imu.ay * 1000) / 1000.0f;
    doc["az"] = roundf(imu.az * 1000) / 1000.0f;
    doc["gravity_x"] = roundf(fs.gravityX * 1000) / 1000.0f;
    doc["gravity_y"] = roundf(fs.gravityY * 1000) / 1000.0f;
    doc["gravity_z"] = roundf(fs.gravityZ * 1000) / 1000.0f;
    doc["gyro_bias_x"] = roundf(fs.gyroBiasX * 10) / 10.0f;
    doc["gyro_bias_y"] = roundf(fs.gyroBiasY * 10) / 10.0f;
    doc["gyro_bias_z"] = roundf(fs.gyroBiasZ * 10) / 10.0f;
    doc["acc_bias_x"] = roundf(fs.accelBiasX * 1000) / 1000.0f;
    doc["acc_bias_y"] = roundf(fs.accelBiasY * 1000) / 1000.0f;
    doc["acc_bias_z"] = roundf(fs.accelBiasZ * 1000) / 1000.0f;
    doc["lin_ax"] = roundf(fs.linAccX * 1000) / 1000.0f;
    doc["lin_ay"] = roundf(fs.linAccY * 1000) / 1000.0f;
    doc["lin_az"] = roundf(fs.linAccZ * 1000) / 1000.0f;
    doc["vel_x"] = roundf(fs.velX * 1000) / 1000.0f;
    doc["vel_y"] = roundf(fs.velY * 1000) / 1000.0f;
    doc["vel_z"] = roundf(fs.velZ * 1000) / 1000.0f;
    doc["pos_x"] = roundf(fs.posX * 1000) / 1000.0f;
    doc["pos_y"] = roundf(fs.posY * 1000) / 1000.0f;
    doc["pos_z"] = roundf(fs.posZ * 1000) / 1000.0f;
    doc["dt_ms"] = roundf(fs.motionDtMs * 100) / 100.0f;
    doc["stationary"] = fs.stationary;
    doc["zupt"] = fs.stationary;
    doc["mode"] = fs.dmpActive ? "mpu_dmp" : "fusion";
    doc["frame"] = "right-handed";
    doc["axes"] = "x:right,y:forward,z:up";
    doc["angles"] = "degrees";
    doc["quat_order"] = "wxyz";

    return serializeJson(doc, buf, bufSize);
}

size_t json_build_raw_gpio(char* buf, size_t bufSize) {
    bool modTemp = mqtt_get_module_temp();
    bool modGyro = mqtt_get_module_gyro();
    bool modCurrent = mqtt_get_module_current();
    bool hasGyro_ = imu_get_addr() != 0;
    const ImuData& imu = imu_get_data();
    const FusionState& fs = fusion_get_state();

    JsonDocument doc;
    doc["ts"] = rtc_get_timestamp();
    doc["up"] = rtc_get_uptime();

    JsonObject gpio = doc["gpio"].to<JsonObject>();
    for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
        char key[10];
        snprintf(key, sizeof(key), "gpio%u", GPIO_PUBLISH_PINS[i]);
        gpio[key] = digitalRead(GPIO_PUBLISH_PINS[i]);
    }

    doc["therm_raw"] = (modTemp && therm_is_valid()) ? therm_get_raw() : 0;

    JsonObject gyroRaw = doc["gyro_raw"].to<JsonObject>();
    if (modGyro && hasGyro_) {
        gyroRaw["x"] = imu.gyroRawX; gyroRaw["y"] = imu.gyroRawY; gyroRaw["z"] = imu.gyroRawZ;
        gyroRaw["addr"] = imu_get_addr();
    } else { gyroRaw["x"]=nullptr; gyroRaw["y"]=nullptr; gyroRaw["z"]=nullptr; gyroRaw["addr"]=nullptr; }

    JsonObject accelRaw = doc["accel_raw"].to<JsonObject>();
    if (modGyro && hasGyro_) {
        accelRaw["x"] = imu.accelRawX; accelRaw["y"] = imu.accelRawY; accelRaw["z"] = imu.accelRawZ;
    } else { accelRaw["x"]=nullptr; accelRaw["y"]=nullptr; accelRaw["z"]=nullptr; }

    JsonObject orient = doc["orientation"].to<JsonObject>();
    if (modGyro && hasGyro_) {
        orient["roll"]    = roundf(fs.roll * 100) / 100.0f;
        orient["pitch"]   = roundf(fs.pitch * 100) / 100.0f;
        orient["yaw"]     = roundf(fs.yaw * 100) / 100.0f;
        orient["heading"] = roundf(fs.heading * 100) / 100.0f;
        orient["q0"] = roundf(fs.q0 * 10000) / 10000.0f;
        orient["q1"] = roundf(fs.q1 * 10000) / 10000.0f;
        orient["q2"] = roundf(fs.q2 * 10000) / 10000.0f;
        orient["q3"] = roundf(fs.q3 * 10000) / 10000.0f;
    } else {
        const char* nk[] = {"roll","pitch","yaw","heading","q0","q1","q2","q3"};
        for (auto k : nk) orient[k] = nullptr;
    }

    JsonObject magRaw = doc["mag_raw"].to<JsonObject>();
    if (modGyro && hasGyro_ && imu.mag_present) {
        magRaw["x"] = imu.magRawX; magRaw["y"] = imu.magRawY; magRaw["z"] = imu.magRawZ;
        magRaw["present"] = true;
    } else { magRaw["x"]=nullptr; magRaw["y"]=nullptr; magRaw["z"]=nullptr; magRaw["present"]=false; }

    JsonObject inaRaw = doc["ina219_raw"].to<JsonObject>();
    inaRaw["bus"]     = modCurrent ? (int)power_get_bus_raw() : 0;
    inaRaw["current"] = modCurrent ? (int)power_get_current_raw() : 0;
    inaRaw["power"]   = modCurrent ? (int)power_get_power_raw() : 0;

    return serializeJson(doc, buf, bufSize);
}
