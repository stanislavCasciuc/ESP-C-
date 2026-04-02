#include "gpio_monitor.h"
#include "config.h"
#include "../sensors/imu.h"
#include "../sensors/thermistor.h"
#include "../sensors/power.h"

#include <Arduino.h>
#include <cstdarg>

static GpioLog sLog = {};
static int8_t sLastState[13]; // GPIO_PUBLISH_PIN_COUNT max
static bool sPrimed = false;
static uint32_t sSeq = 0;
static uint8_t sPhase = 0;

static void appendLine(const char* fmt, ...) {
    char line[OLED_LOG_LINE_LEN + 1];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    line[OLED_LOG_LINE_LEN] = '\0';
    strncpy(sLog.lines[sLog.head], line, OLED_LOG_LINE_LEN);
    sLog.lines[sLog.head][OLED_LOG_LINE_LEN] = '\0';
    sLog.head = (sLog.head + 1) % OLED_LOG_LINES;
    sLog.hasData = true;
}

static const char* gpioShortName(uint8_t pin) {
    switch(pin) {
        case 3: return "BTN+"; case 4: return "BTN-";
        case 8: return "SDA";  case 9: return "SCL";
        default: return "IO";
    }
}

void gpio_monitor_init() {
    for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) sLastState[i] = -1;
}

void gpio_monitor_update() {
    sSeq++;
    if (!sPrimed) {
        for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++)
            sLastState[i] = digitalRead(GPIO_PUBLISH_PINS[i]);
        appendLine("Logger pornit");
        sPrimed = true;
        return;
    }

    int changes = 0;
    for (size_t i = 0; i < GPIO_PUBLISH_PIN_COUNT; i++) {
        uint8_t pin = GPIO_PUBLISH_PINS[i];
        int8_t now = digitalRead(pin);
        if (now != sLastState[i]) {
            appendLine("#%lu G%02u %s %s", (unsigned long)(sSeq%1000), pin,
                       gpioShortName(pin), now?"ON":"OFF");
            sLastState[i] = now;
            changes++;
            if (changes >= 2) break;
        }
    }
    if (changes > 0) return;

    const ImuData& imu = imu_get_data();
    switch (sPhase % 3) {
        case 0: appendLine("TH RAW:%4u T:%4.1f", therm_get_raw(), therm_read()); break;
        case 1: appendLine("GYR X:%4.0f Y:%4.0f", imu.gx, imu.gy); break;
        default: appendLine("GYR Z:%4.0f INA:%5u", imu.gz, power_get_bus_raw()); break;
    }
    sPhase++;
}

const GpioLog& gpio_get_log() { return sLog; }
