#pragma once

#include <cstdint>

void ws_init();
void ws_loop();
uint16_t ws_client_count();
void ws_broadcast_imu();
