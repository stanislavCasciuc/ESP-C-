#pragma once

#include <PubSubClient.h>

void mqtt_init(PubSubClient& client);
void mqtt_connect();
void mqtt_publish_telemetry();
void mqtt_publish_raw();
void mqtt_publish_state();
void mqtt_set_modules(bool temp, bool gyro, bool cpu, bool current, bool cpuStress);
bool mqtt_get_module_temp();
bool mqtt_get_module_gyro();
bool mqtt_get_module_cpu();
bool mqtt_get_module_current();
bool mqtt_get_module_cpu_stress();
