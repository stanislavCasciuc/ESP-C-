#include "mqtt_client.h"
#include "config.h"
#include "../data/json_builder.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <algorithm>

static PubSubClient* sMqtt = nullptr;
static String sMqttClientId;

// Module flags
static bool sModTemp = true, sModGyro = true, sModCPU = true, sModCurrent = true;
static volatile bool sModCpuStress = false;

bool mqtt_get_module_temp()      { return sModTemp; }
bool mqtt_get_module_gyro()      { return sModGyro; }
bool mqtt_get_module_cpu()       { return sModCPU; }
bool mqtt_get_module_current()   { return sModCurrent; }
bool mqtt_get_module_cpu_stress(){ return sModCpuStress; }

void mqtt_set_modules(bool temp, bool gyro, bool cpu, bool current, bool cpuStress) {
    sModTemp = temp; sModGyro = gyro; sModCPU = cpu;
    sModCurrent = current; sModCpuStress = cpuStress;
}

static void onMessage(char* topic, byte* payload, unsigned int length) {
    (void)topic;
    char msg[128];
    size_t len = min((unsigned int)(sizeof(msg) - 1), length);
    memcpy(msg, payload, len);
    msg[len] = '\0';

    JsonDocument doc;
    if (deserializeJson(doc, msg) != DeserializationError::Ok) return;

    const char* mod = doc["module"];
    if (!mod) return;
    bool en = doc["enabled"] | false;

    if      (strcmp(mod, "temperature") == 0) sModTemp = en;
    else if (strcmp(mod, "gyro") == 0)        sModGyro = en;
    else if (strcmp(mod, "cpu") == 0)         sModCPU = en;
    else if (strcmp(mod, "current") == 0)     sModCurrent = en;
    else if (strcmp(mod, "cpu_stress") == 0) { sModCpuStress = en; if (en) sModCPU = true; }

    mqtt_publish_state();
}

void mqtt_init(PubSubClient& client) {
    sMqtt = &client;
    sMqttClientId = "esp32-hs-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    sMqtt->setServer(MQTT_BROKER, MQTT_PORT);
    sMqtt->setCallback(onMessage);
    sMqtt->setBufferSize(2048);
}

void mqtt_connect() {
    if (!sMqtt || WiFi.status() != WL_CONNECTED) return;
    if (sMqtt->connect(sMqttClientId.c_str(), MQTT_USER, MQTT_PASS_MQTT)) {
        sMqtt->subscribe(MQTT_CMD_TOPIC);
        sMqtt->publish(MQTT_TOPIC, "{\"status\":\"online\"}");
        mqtt_publish_state();
    }
}

void mqtt_publish_state() {
    if (!sMqtt) return;
    char buf[128];
    JsonDocument doc;
    JsonObject m = doc["modules"].to<JsonObject>();
    m["temperature"] = sModTemp;
    m["gyro"]        = sModGyro;
    m["cpu"]         = sModCPU;
    m["current"]     = sModCurrent;
    m["cpu_stress"]  = sModCpuStress;
    serializeJson(doc, buf, sizeof(buf));
    sMqtt->publish(MQTT_STATE_TOPIC, buf);
}

void mqtt_publish_telemetry() {
    if (!sMqtt || !sMqtt->connected()) return;
    char json[1024];
    size_t len = json_build_telemetry(json, sizeof(json));
    if (len < sizeof(json)) sMqtt->publish(MQTT_TOPIC, json);
}

void mqtt_publish_raw() {
    if (!sMqtt || !sMqtt->connected()) return;
    char json[1536];
    size_t len = json_build_raw_gpio(json, sizeof(json));
    if (len < sizeof(json)) sMqtt->publish(MQTT_RAW_TOPIC, json);
}
