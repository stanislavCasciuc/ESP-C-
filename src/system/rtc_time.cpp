#include "rtc_time.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <time.h>

static RTC_DS1307 sRtc;
static bool sHasRTC = false;
static uint8_t sRtcAddr = 0;
static char sTimestamp[20] = "0000-00-00 00:00:00";
static uint32_t sUptime = 0;

static bool i2cProbeAddr(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

static bool systemTimeValid() {
    return time(nullptr) > 1704067200;
}

static uint8_t bcdToBin(uint8_t v) { return (v & 0x0F) + ((v >> 4) * 10); }
static uint8_t binToBcd(uint8_t v) { return ((v/10) << 4) | (v%10); }

static bool i2cReadRegsLocal(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (uint8_t)len) < len) return false;
    for (size_t i = 0; i < len; i++) data[i] = Wire.read();
    return true;
}

static bool rtcReadDateTime(int& y, int& mo, int& d, int& h, int& mi, int& s) {
    if (!sHasRTC || sRtcAddr == 0) return false;
    uint8_t regs[7] = {0};
    if (!i2cReadRegsLocal(sRtcAddr, 0x00, regs, 7)) return false;
    s  = bcdToBin(regs[0] & 0x7F);
    mi = bcdToBin(regs[1] & 0x7F);
    h  = bcdToBin(regs[2] & 0x3F);
    d  = bcdToBin(regs[4] & 0x3F);
    mo = bcdToBin(regs[5] & 0x1F);
    y  = 2000 + bcdToBin(regs[6]);
    return y >= 2020 && y <= 2099 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31;
}

bool rtc_init() {
    sRtcAddr = 0;
    if (i2cProbeAddr(I2C_ADDR_RTC)) sRtcAddr = I2C_ADDR_RTC;
    else if (i2cProbeAddr(I2C_ADDR_RTC_ALT)) sRtcAddr = I2C_ADDR_RTC_ALT;

    sHasRTC = (sRtcAddr != 0);
    if (sHasRTC && sRtcAddr == I2C_ADDR_RTC) {
        sRtc.begin();
        if (!sRtc.isrunning()) {
            sRtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
    }
    return sHasRTC;
}

void rtc_update_timestamp() {
    sUptime = millis() / 1000;

    if (systemTimeValid()) {
        struct tm ti;
        if (getLocalTime(&ti, 50)) {
            snprintf(sTimestamp, sizeof(sTimestamp), "%04d-%02d-%02d %02d:%02d:%02d",
                     ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
            return;
        }
    }

    if (sHasRTC) {
        int y, mo, d, h, mi, s;
        if (rtcReadDateTime(y, mo, d, h, mi, s)) {
            snprintf(sTimestamp, sizeof(sTimestamp), "%04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
            return;
        }
    }

    uint32_t sec = millis() / 1000;
    snprintf(sTimestamp, sizeof(sTimestamp), "1970-01-01 %02lu:%02lu:%02lu",
             (unsigned long)(sec/3600)%24, (unsigned long)(sec/60)%60, (unsigned long)sec%60);
}

void rtc_sync_from_system() {
    if (!sHasRTC || !systemTimeValid()) return;
    struct tm ti;
    if (!getLocalTime(&ti, 100)) return;
    // Write directly via RTC lib
    if (sRtcAddr == I2C_ADDR_RTC) {
        sRtc.adjust(DateTime(ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday,
                             ti.tm_hour, ti.tm_min, ti.tm_sec));
    }
}

const char* rtc_get_timestamp() { return sTimestamp; }
uint32_t rtc_get_uptime() { return sUptime; }
