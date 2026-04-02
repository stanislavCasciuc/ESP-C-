#pragma once

#include <Adafruit_SSD1306.h>

void display_init(Adafruit_SSD1306& oled);
void display_draw_page();
bool display_is_available();
void display_set_page(int8_t p);
int8_t display_get_page();
