#pragma once

#include <cstdint>

// Initialize thermistor ADC pin
void therm_init();

// Read temperature in Celsius (filtered, calibrated). Returns last valid on error.
float therm_read();

// Check if last reading was valid
bool therm_is_valid();

// Get raw ADC value from last reading
uint16_t therm_get_raw();
