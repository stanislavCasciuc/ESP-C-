#include "wifi_manager.h"
#include "config.h"

#include <Arduino.h>
#include <WiFi.h>
#include <sMQTTBroker.h>
#include <Adafruit_SSD1306.h>

// External references to OLED (set by main)
extern Adafruit_SSD1306* g_oled;
extern bool g_hasOLED;
extern volatile bool isrNextFlag;
extern volatile bool isrPrevFlag;
extern uint32_t gIgnoreButtonsUntilMs;

static sMQTTBroker sLocalBroker;
static bool sOfflineActive = false;
static bool sLocalBrokerStarted = false;
static bool sWasWifiConnected = false;

bool wifi_is_offline_active()       { return sOfflineActive; }
bool wifi_is_local_broker_started() { return sLocalBrokerStarted; }

static bool consumeSkipRequest() {
    if (!isrNextFlag && !isrPrevFlag) return false;
    isrNextFlag = false;
    isrPrevFlag = false;
    gIgnoreButtonsUntilMs = millis() + 800;
    return true;
}

static bool connectNetwork(const char* ssid, const char* pass,
                           uint8_t timeoutSec, size_t idx, size_t total,
                           bool& skipReq) {
    skipReq = false;
    WiFi.disconnect(true, true);
    delay(120);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_13dBm);
    WiFi.begin(ssid, pass);

    Serial.printf("[WiFi] Trying %s (%u/%u)\n", ssid, (unsigned)(idx+1), (unsigned)total);

    uint32_t deadline = millis() + ((uint32_t)timeoutSec * 1000UL);
    uint8_t lastShownSec = 255;

    while (WiFi.status() != WL_CONNECTED) {
        if (consumeSkipRequest()) { skipReq = true; WiFi.disconnect(false,false); break; }
        uint32_t now = millis();
        if ((int32_t)(deadline - now) <= 0) break;

        wl_status_t st = WiFi.status();
        if (st == WL_NO_SSID_AVAIL || st == WL_CONNECT_FAILED || st == WL_CONNECTION_LOST) break;

        uint8_t secLeft = (uint8_t)((deadline - now + 999UL) / 1000UL);
        if (g_hasOLED && g_oled && secLeft != lastShownSec) {
            lastShownSec = secLeft;
            g_oled->clearDisplay();
            g_oled->fillRect(0, 0, 128, OLED_TITLE_H, SSD1306_WHITE);
            g_oled->setTextColor(SSD1306_BLACK);
            g_oled->setTextSize(1);
            g_oled->setCursor(2, OLED_TITLE_TEXT_Y);
            g_oled->printf("WiFi %u/%u", (unsigned)(idx+1), (unsigned)total);
            g_oled->setTextColor(SSD1306_WHITE);
            g_oled->setCursor(0, 20);
            g_oled->print(ssid);
            g_oled->setCursor(0, 56);
            g_oled->print("BTN=SKIP");
            char secText[4];
            snprintf(secText, sizeof(secText), "%u", secLeft);
            g_oled->setTextSize(4);
            int x = (128 - ((int)strlen(secText) * 24)) / 2;
            if (x < 0) x = 0;
            g_oled->setCursor(x, 30);
            g_oled->print(secText);
            g_oled->display();
        }
        delay(80);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool wifi_connect_from_list() {
    if (WIFI_NETWORK_COUNT == 0) return false;
    for (size_t i = 0; i < WIFI_NETWORK_COUNT; i++) {
        bool skip = false;
        if (connectNetwork(WIFI_SSIDS[i], WIFI_PASSWORDS[i],
                           WIFI_ATTEMPT_TIMEOUT_SEC, i, WIFI_NETWORK_COUNT, skip))
            return true;
    }
    return false;
}

void wifi_sync_ntp() {
    if (WiFi.status() != WL_CONNECTED) return;
    configTzTime("EET-2EEST,M3.5.0/3,M10.5.0/4", "pool.ntp.org", "time.nist.gov");
    uint8_t tries = 0;
    while (time(nullptr) < 1704067200 && tries < 25) { delay(200); tries++; }
}

void wifi_start_offline_services() {
    if (sOfflineActive) return;
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, false, AP_MAX_CONN);
    Serial.printf("[AP] SSID:%s IP:%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    if (!sLocalBrokerStarted) {
        sLocalBrokerStarted = sLocalBroker.init(1883, true);
    }
    sOfflineActive = true;
}

void wifi_stop_offline_services() {
    if (!sOfflineActive) return;
    WiFi.softAPdisconnect(true);
    sOfflineActive = false;
}

void wifi_update_connectivity() {
    bool online = (WiFi.status() == WL_CONNECTED);
    if (online && !sWasWifiConnected) {
        wifi_sync_ntp();
    }
    sWasWifiConnected = online;
    if (!online) wifi_start_offline_services();
    else         wifi_stop_offline_services();
}

// Provide access to local broker for offline MQTT publishing
sMQTTBroker& wifi_get_local_broker() { return sLocalBroker; }
