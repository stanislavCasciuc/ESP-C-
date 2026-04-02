#pragma once

#include <cstdint>

// Initialize RTC (DS1307). Returns true if found.
bool rtc_init();

// Update timestamp from NTP/RTC/uptime fallback
void rtc_update_timestamp();

// Get current timestamp string and uptime
const char* rtc_get_timestamp();
uint32_t    rtc_get_uptime();

// Write RTC time from NTP
void rtc_sync_from_system();
