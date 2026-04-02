#pragma once

#include <cstdint>

bool  power_init();
void  power_read();
bool  power_is_available();

float    power_get_voltage();
float    power_get_current_ma();
float    power_get_power_mw();
uint16_t power_get_bus_raw();
int16_t  power_get_current_raw();
uint16_t power_get_power_raw();
