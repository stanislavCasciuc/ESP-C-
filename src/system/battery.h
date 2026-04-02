#pragma once

#include <cstdint>

void battery_load_state();
void battery_update(float currentMA, float voltage, uint32_t dtMs, bool moduleCurrentActive);
void battery_persist_if_due(uint32_t nowMs);

float battery_get_percent();
float battery_get_used_mah();
float battery_get_life_min();
bool  battery_is_present();
int   battery_get_shown_pct();
