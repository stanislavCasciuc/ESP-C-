#include "thermistor.h"
#include "config.h"

#include <Arduino.h>
#include <cmath>
#include "power.h"

static float sFiltered = 0.0f;
static bool sFilterSeeded = false;
static bool sValid = false;
static uint16_t sRaw = 0;

static bool tempFromResistance(float rNtc, float r25, float& tempC) {
    const float t0K = T_NOMINAL + 273.15f;
    const float invTK = (1.0f / t0K) + (logf(rNtc / r25) / B_COEFF);
    if (!isfinite(invTK) || invTK <= 0.0f) return false;
    tempC = (1.0f / invTK) - 273.15f;
    if (!isfinite(tempC) || tempC < -40.0f || tempC > 125.0f) return false;
    return true;
}

void therm_init() {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_THERM, ADC_11db);
}

float therm_read() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum += (uint16_t)analogRead(PIN_THERM);
        delayMicroseconds(250);
    }
    sRaw = (uint16_t)(sum / 8U);
    float adc = (float)sRaw;

    if (adc <= 1.0f || adc >= (ADC_MAX_F - 1.0f)) {
        sValid = false;
        return sFilterSeeded ? sFiltered : -99.0f;
    }

    const float vAdc = (adc / ADC_MAX_F) * THERM_ADC_REF_V;

    float vSupply = THERM_SUPPLY_FALLBACK_V;
    if (power_is_available()) {
        float v = power_get_voltage();
        if (isfinite(v) && v > 2.5f && v < 5.5f) vSupply = v;
    }

    if (vAdc <= 0.001f || vAdc >= (vSupply - 0.001f)) {
        sValid = false;
        return sFilterSeeded ? sFiltered : -99.0f;
    }

    float rNtc = R_FIXED * (vAdc / (vSupply - vAdc));
    if (!isfinite(rNtc) || rNtc < 100.0f || rNtc > 1000000.0f) {
        sValid = false;
        return sFilterSeeded ? sFiltered : -99.0f;
    }

    float tempC = NAN;
    if (!tempFromResistance(rNtc, R_NOMINAL, tempC)) {
        sValid = false;
        return sFilterSeeded ? sFiltered : -99.0f;
    }

    if (!isfinite(tempC) || tempC < -40.0f || tempC > 125.0f) {
        sValid = false;
        return sFilterSeeded ? sFiltered : -99.0f;
    }

    tempC = tempC * THERM_CAL_GAIN + THERM_CAL_OFFSET_C;

    if (sFilterSeeded && fabsf(tempC - sFiltered) > 20.0f) {
        sValid = true;
        return sFiltered;
    }

    if (!sFilterSeeded) {
        sFiltered = tempC;
        sFilterSeeded = true;
    } else {
        sFiltered = sFiltered + THERM_FILTER_ALPHA * (tempC - sFiltered);
    }

    sValid = true;
    return sFiltered;
}

bool therm_is_valid() { return sValid; }
uint16_t therm_get_raw() { return sRaw; }
