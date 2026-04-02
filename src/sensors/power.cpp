#include "power.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

static Adafruit_INA219 sIna;
static bool sAvailable = false;
static float sVoltage = 0, sCurrentMA = 0, sPowerMW = 0;
static uint16_t sBusRaw = 0;
static int16_t  sCurrRaw = 0;
static uint16_t sPowRaw = 0;

static bool readReg16(uint8_t reg, uint16_t& out) {
    Wire.beginTransmission(I2C_ADDR_INA219);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)I2C_ADDR_INA219, (uint8_t)2);
    if (Wire.available() < 2) return false;
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    out = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return true;
}

static bool i2cProbeAddr(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

bool power_init() {
    sAvailable = i2cProbeAddr(I2C_ADDR_INA219);
    if (sAvailable) {
        sAvailable = sIna.begin();
        if (sAvailable) {
            sIna.setCalibration_16V_400mA();
        }
    }
    return sAvailable;
}

void power_read() {
    if (!sAvailable) return;

    uint16_t raw = 0;
    if (readReg16(0x02, raw)) sBusRaw = raw;
    if (readReg16(0x04, raw)) sCurrRaw = (int16_t)raw;
    if (readReg16(0x03, raw)) sPowRaw = raw;

    sVoltage   = sIna.getBusVoltage_V();
    sCurrentMA = sIna.getCurrent_mA();
    sPowerMW   = sIna.getPower_mW();

    if (isnan(sVoltage)   || isinf(sVoltage))   sVoltage   = 0.0f;
    if (isnan(sCurrentMA) || isinf(sCurrentMA))  sCurrentMA = 0.0f;
    if (isnan(sPowerMW)   || isinf(sPowerMW))    sPowerMW   = 0.0f;

    if (fabsf(sCurrentMA) < BATT_CURRENT_DEADBAND_MA) sCurrentMA = 0.0f;
}

bool  power_is_available()    { return sAvailable; }
float power_get_voltage()     { return sVoltage; }
float power_get_current_ma()  { return sCurrentMA; }
float power_get_power_mw()    { return sPowerMW; }
uint16_t power_get_bus_raw()  { return sBusRaw; }
int16_t  power_get_current_raw() { return sCurrRaw; }
uint16_t power_get_power_raw()   { return sPowRaw; }
