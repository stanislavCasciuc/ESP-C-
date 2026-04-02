#pragma once

#include <cstddef>

size_t json_build_telemetry(char* buf, size_t bufSize);
size_t json_build_imu(char* buf, size_t bufSize);
size_t json_build_raw_gpio(char* buf, size_t bufSize);
