#include "display.h"
#include "config.h"
#include "../sensors/imu.h"
#include "../sensors/thermistor.h"
#include "../sensors/power.h"
#include "../motion/fusion.h"
#include "../network/mqtt_client.h"
#include "../network/wifi_manager.h"
#include "../system/cpu_monitor.h"
#include "../system/battery.h"
#include "../system/rtc_time.h"
#include "../data/gpio_monitor.h"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <qrcode.h>
#include <cmath>

static Adafruit_SSD1306* sOled = nullptr;
static bool sAvailable = false;
static int8_t sPage = 0;

extern PubSubClient g_mqtt;

// I2C scan state
static uint8_t sI2cFound[I2C_SCAN_MAX_DEVICES];
static uint8_t sI2cFoundCount = 0;
static bool sI2cOverflow = false;
static uint32_t sLastScanMs = 0;

static bool i2cProbeLocal(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

static const char* i2cDevName(uint8_t addr) {
    switch(addr) {
        case 0x3C: return "SSD1306"; case 0x0C: return "AK8963";
        case 0x40: return "INA219";  case 0x50: return "AT24C32";
        case 0x60: return "RTC";     case 0x68: return "DS1307";
        case 0x69: return "MPU6500"; case 0x6A: return "LSM6DS3";
        default: return "Unknown";
    }
}

void scanI2c() {
    sI2cFoundCount = 0; sI2cOverflow = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            if (sI2cFoundCount < I2C_SCAN_MAX_DEVICES) sI2cFound[sI2cFoundCount++] = a;
            else sI2cOverflow = true;
        }
    }
    sLastScanMs = millis();
}

// Drawing helpers
static void drawTitleBar(const char* title) {
    sOled->fillRect(0, 0, 128, OLED_TITLE_H, SSD1306_WHITE);
    sOled->setTextColor(SSD1306_BLACK);
    sOled->setTextSize(1);
    int x = (102 - (int)strlen(title) * 6) / 2;
    if (x < 1) x = 1;
    sOled->setCursor(x, OLED_TITLE_TEXT_Y);
    sOled->print(title);
    sOled->setCursor(103, OLED_TITLE_TEXT_Y);
    sOled->printf("%d/%d", sPage + 1, NUM_PAGES);
    sOled->setTextColor(SSD1306_WHITE);
}

static void drawBar(int x, int y, int w, int h, float pct) {
    pct = constrain(pct, 0, 100);
    sOled->drawRoundRect(x, y, w, h, 3, SSD1306_WHITE);
    int fw = (int)(pct / 100.0f * (w-4));
    if (fw > 0) sOled->fillRoundRect(x+2, y+2, fw, h-4, 2, SSD1306_WHITE);
}

static void drawCentered(const char* text, int y, int sz) {
    sOled->setTextSize(sz);
    int x = (128 - (int)strlen(text)*6*sz)/2;
    if (x<0) x=0;
    sOled->setCursor(x, y);
    sOled->print(text);
}

static void printDegC() { sOled->write(248); sOled->print("C"); }

// Pages
static void pageDashboard() {
    drawTitleBar("MONITOR");
    const FusionState& fs = fusion_get_state();
    const ImuData& imu = imu_get_data();
    float temp = therm_read();
    float cpuLoad = cpu_get_load();
    float battPct = battery_get_percent();

    sOled->setTextColor(SSD1306_WHITE); sOled->setTextSize(1);
    const char* ts = rtc_get_timestamp();
    if (strlen(ts) >= 19) {
        sOled->setCursor(0, 18); char hms[9]; memcpy(hms, ts+11, 8); hms[8]=0;
        sOled->printf("ORA: %s", hms);
    }
    sOled->setCursor(0,28);  sOled->printf("T:%.1f",temp); printDegC();
    sOled->setCursor(68,28); sOled->printf("G:%.0f",imu.gyroAbs);
    sOled->setCursor(0,38);  sOled->printf("P:%.0fmW",power_get_power_mw());
    sOled->setCursor(68,38); sOled->printf("C:%.0fmA",power_get_current_ma());
    sOled->setCursor(0,48);  sOled->printf("R:%ddBm",(int16_t)WiFi.RSSI());
    sOled->setCursor(68,48); sOled->printf("CPU:%d%%",(int)roundf(cpuLoad));
    drawBar(0, 56, 104, 8, battPct);
    sOled->setCursor(108, 56); sOled->printf("%.0f%%", battPct);
}

static void pageTemp() {
    drawTitleBar("TEMPERATURA");
    char buf[10]; snprintf(buf, sizeof(buf), "%.1f", therm_read());
    int numW = strlen(buf)*18;
    int sx = (128-numW-14)/2; if(sx<0) sx=0;
    sOled->setTextSize(3); sOled->setCursor(sx, 18); sOled->print(buf);
    sOled->setTextSize(2); sOled->setCursor(sx+numW+6, 22); sOled->print("C");
    sOled->drawCircle(sx+numW+2, 23, 1, SSD1306_WHITE);
    drawBar(4, 48, 120, 8, constrain((therm_read()+10)/60*100, 0, 100));
}

static void pageGyro() {
    drawTitleBar("GIROSCOP");
    const ImuData& imu = imu_get_data();
    sOled->setTextSize(1);
    sOled->setCursor(0,18); sOled->printf("X:%7.2f dps", imu.gx);
    sOled->setCursor(0,28); sOled->printf("Y:%7.2f dps", imu.gy);
    sOled->setCursor(0,38); sOled->printf("Z:%7.2f dps", imu.gz);
    sOled->setCursor(0,48); sOled->printf("ABS:%6.2f dps", imu.gyroAbs);
    drawBar(4, 56, 120, 8, constrain(imu.gyroAbs/10, 0, 100));
}

static void pageWiFi() {
    if (wifi_is_offline_active()) {
        drawTitleBar("HOTSPOT OFFLINE");
        sOled->setTextSize(1);
        sOled->setCursor(0,18); sOled->print("SSID:");
        sOled->setCursor(0,28); sOled->print(AP_SSID);
        sOled->setCursor(0,40); sOled->print("Pass:");
        sOled->setCursor(0,50); sOled->print(AP_PASS);

        char wifiQr[96];
        snprintf(wifiQr, sizeof(wifiQr), "WIFI:T:WPA;S:%s;P:%s;;", AP_SSID, AP_PASS);
        QRCode qrcode;
        uint8_t qrData[qrcode_getBufferSize(1)];
        if (qrcode_initText(&qrcode, qrData, 1, ECC_LOW, wifiQr) == 0) {
            int qrSize = qrcode.size;
            int x0 = 128-qrSize-2, y0 = 18;
            for (uint8_t y=0; y<qrcode.size; y++)
                for (uint8_t x=0; x<qrcode.size; x++)
                    if (qrcode_getModule(&qrcode, x, y))
                        sOled->fillRect(x0+x, y0+y, 1, 1, SSD1306_WHITE);
        }
        return;
    }

    drawTitleBar("RETEA + MQTT");
    sOled->setTextSize(1);
    sOled->setCursor(0,18); sOled->printf("SSID: %s", WiFi.SSID().c_str());
    sOled->setCursor(0,28); sOled->printf("IP:   %s", WiFi.localIP().toString().c_str());
    sOled->setCursor(0,38); sOled->printf("RSSI: %d dBm", (int)WiFi.RSSI());
    const char* ts = rtc_get_timestamp();
    sOled->setCursor(0,48); sOled->printf("Data: %.10s", strlen(ts)>=10?ts:"----------");
    sOled->setCursor(0,58); sOled->printf("MQTT:%s", g_mqtt.connected()?"OK":"Off");

    int bars = 0;
    if (WiFi.status() == WL_CONNECTED) bars = constrain(((int)WiFi.RSSI()+100)/10, 0, 4);
    for (int i=0; i<4; i++) {
        int bh=4+i*3, by=54-bh, bx=100+i*7;
        if(i<bars) sOled->fillRect(bx,by,5,bh,SSD1306_WHITE);
        else sOled->drawRect(bx,by,5,bh,SSD1306_WHITE);
    }
}

static void pageCPU() {
    drawTitleBar("INC. CPU");
    float load = cpu_get_load();
    char buf[10]; snprintf(buf, sizeof(buf), "%d", (int)roundf(load));
    int numW=strlen(buf)*18; int sx=(128-numW-14)/2; if(sx<0) sx=0;
    sOled->setTextSize(3); sOled->setCursor(sx,18); sOled->print(buf);
    sOled->setTextSize(2); sOled->setCursor(sx+numW+2,22); sOled->print("%");
    drawBar(4, 48, 120, 8, load);
}

static void pageVoltage() {
    drawTitleBar("TENSIUNE");
    float v = power_get_voltage();
    char buf[10]; snprintf(buf, sizeof(buf), "%.2f", v);
    int numW=strlen(buf)*18; int sx=(128-numW-14)/2; if(sx<0) sx=0;
    sOled->setTextSize(3); sOled->setCursor(sx,18); sOled->print(buf);
    sOled->setTextSize(2); sOled->setCursor(sx+numW+2,22); sOled->print("V");
    char pow[20]; snprintf(pow, sizeof(pow), "Put: %.0f mW", power_get_power_mw());
    drawCentered(pow, 42, 1);
    drawBar(4, 52, 120, 8, constrain((v-3)/1.2f*100, 0, 100));
}

static void pageCurrent() {
    drawTitleBar("CURENT");
    char buf[16]; snprintf(buf, sizeof(buf), "%.0f mA", power_get_current_ma());
    drawCentered(buf, 18, 2);
    char total[22]; snprintf(total, sizeof(total), "CP: %.2f mAh", battery_get_used_mah());
    drawCentered(total, 38, 1);
    char pow[20]; snprintf(pow, sizeof(pow), "Put: %.0f mW", power_get_power_mw());
    drawCentered(pow, 48, 1);
}

static void pageBattery() {
    drawTitleBar("BATERIE");
    float pct = battery_get_percent();
    char buf[10]; snprintf(buf, sizeof(buf), "%.0f", pct);
    int numW=strlen(buf)*18; int sx=(128-numW-14)/2; if(sx<0) sx=0;
    sOled->setTextSize(3); sOled->setCursor(sx,18); sOled->print(buf);
    sOled->setTextSize(2); sOled->setCursor(sx+numW+2,22); sOled->print("%");
    sOled->setTextSize(1);
    float life = battery_get_life_min();
    if (battery_is_present() && life > 0) {
        char est[20]; snprintf(est, sizeof(est), "Est: %dh %dm", (int)life/60, (int)life%60);
        drawCentered(est, 40, 1);
    } else if (!battery_is_present()) drawCentered("Est: -- (fara bat)", 40, 1);
    else drawCentered("Est: -- (fara sarc)", 40, 1);
    char used[24]; snprintf(used, sizeof(used), "CP: %.2f mAh", battery_get_used_mah());
    drawCentered(used, 50, 1);
}

static void pageRawLog() {
    bool summary = ((millis()/5000UL)%2UL)==1;
    drawTitleBar(summary ? "RAW SUMMARY" : "RAW EVENTS");
    sOled->setTextSize(1);
    const ImuData& imu = imu_get_data();

    if (summary) {
        int gpioHigh = 0;
        for (size_t i=0; i < GPIO_PUBLISH_PIN_COUNT; i++)
            if (digitalRead(GPIO_PUBLISH_PINS[i])) gpioHigh++;
        sOled->setCursor(0,18); sOled->printf("THM RAW:%4u", therm_get_raw());
        sOled->setCursor(0,27); sOled->printf("GYR X:%5d", imu.gyroRawX);
        sOled->setCursor(0,36); sOled->printf("GYR Y:%5d", imu.gyroRawY);
        sOled->setCursor(0,45); sOled->printf("GYR Z:%5d", imu.gyroRawZ);
        sOled->setCursor(0,54); sOled->printf("GPIO HIGH:%2d/%2d", gpioHigh, (int)GPIO_PUBLISH_PIN_COUNT);
        return;
    }

    const GpioLog& log = gpio_get_log();
    if (!log.hasData) { drawCentered("Astept evenimente...", 28, 1); return; }
    for (uint8_t row=0; row<OLED_LOG_LINES; row++) {
        uint8_t idx = (log.head + row) % OLED_LOG_LINES;
        sOled->setCursor(0, 18+row*9);
        if (row == OLED_LOG_LINES-1) sOled->print('>'); else sOled->print(' ');
        sOled->print(log.lines[idx]);
    }
}

static void pageI2CScanner() {
    drawTitleBar("I2C SCAN");
    sOled->setTextSize(1);
    if (sI2cFoundCount == 0) {
        sOled->setCursor(0,18); sOled->print("No devices found");
        sOled->setCursor(0,28); sOled->print("Check SDA/SCL/wiring");
        return;
    }
    sOled->setCursor(0,18); sOled->printf("Found: %u", sI2cFoundCount);
    if (sI2cOverflow) sOled->print("+");
    uint8_t maxRows = 4;
    uint8_t pages = (sI2cFoundCount+maxRows-1)/maxRows;
    uint8_t pageIdx = (pages>1) ? (uint8_t)((millis()/2000UL)%pages) : 0;
    uint8_t first = pageIdx*maxRows;
    if (pages>1) { sOled->setCursor(88,18); sOled->printf("%u/%u", pageIdx+1, pages); }
    for (uint8_t i=0; i<maxRows; i++) {
        uint8_t idx = first+i;
        if (idx >= sI2cFoundCount) break;
        sOled->setCursor(0, 28+i*9);
        sOled->printf("0x%02X %-8s", sI2cFound[idx], i2cDevName(sI2cFound[idx]));
    }
}

// Public interface
void display_init(Adafruit_SSD1306& oled) {
    sOled = &oled;
    sAvailable = i2cProbeLocal(I2C_ADDR_OLED);
    if (sAvailable) {
        sAvailable = sOled->begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_OLED);
        if (sAvailable) {
            sOled->clearDisplay();
            sOled->setTextColor(SSD1306_WHITE);
            sOled->setTextSize(1);
            sOled->setCursor(0,0);
            sOled->println("Booting...");
            sOled->display();
        }
    }
    scanI2c();
}

void display_draw_page() {
    if (!sAvailable || !sOled) return;
    sOled->clearDisplay();
    switch(sPage) {
        case 0: pageDashboard(); break;
        case 1: pageTemp(); break;
        case 2: pageGyro(); break;
        case 3: pageWiFi(); break;
        case 4: pageCPU(); break;
        case 5: pageVoltage(); break;
        case 6: pageCurrent(); break;
        case 7: pageBattery(); break;
        case 8: pageRawLog(); break;
        case 9: pageI2CScanner(); break;
    }
    sOled->display();

    // Background I2C scan refresh
    if (millis() - sLastScanMs >= 3000) scanI2c();
}

bool display_is_available() { return sAvailable; }
void display_set_page(int8_t p) { sPage = p; }
int8_t display_get_page() { return sPage; }
