#pragma once

#include <cstdint>

struct GpioLog {
    char lines[5][22];  // OLED_LOG_LINES x (OLED_LOG_LINE_LEN+1)
    uint8_t head;
    bool hasData;
};

void gpio_monitor_init();
void gpio_monitor_update();
const GpioLog& gpio_get_log();
