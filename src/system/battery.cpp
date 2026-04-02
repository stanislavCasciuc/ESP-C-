#include "battery.h"
#include "config.h"

#include <Arduino.h>
#include <Preferences.h>
#include <cmath>
#include <algorithm>

static Preferences sPrefs;
static float sTotalMAh = 0;
static float sSocPct = 100.0f;
static int sShownPct = 100;
static float sBattLife = -1.0f;
static bool sPresent = true;
static bool sHasPersisted = false;
static bool sBootstrapDone = false;
static uint32_t sLastPersistMs = 0;
static uint32_t sRestAccumMs = 0;
static uint32_t sBattLowAccumMs = 0;
static uint32_t sBattHighAccumMs = 0;

static float ocvToSoc(float v) {
    static const float vPts[] = {4.20f,4.10f,4.00f,3.90f,3.80f,3.70f,3.60f,3.50f,3.40f,3.30f,3.20f};
    static const float sPts[] = {100,90,75,60,45,30,18,10,5,2,0};
    if (v >= vPts[0]) return 100;
    if (v <= vPts[10]) return 0;
    for (int i = 0; i < 10; i++) {
        if (v <= vPts[i] && v >= vPts[i+1]) {
            float t = (v - vPts[i+1]) / (vPts[i] - vPts[i+1]);
            return sPts[i+1] + t * (sPts[i] - sPts[i+1]);
        }
    }
    return 0;
}

void battery_load_state() {
    if (!sPrefs.begin(PREF_NS, false)) return;
    sHasPersisted = sPrefs.isKey(PREF_KEY_USED_MAH);
    if (sHasPersisted) sTotalMAh = sPrefs.getFloat(PREF_KEY_USED_MAH, sTotalMAh);
    sPrefs.end();
    sTotalMAh = constrain(sTotalMAh, 0.0f, BATTERY_CAPACITY);
    if (!sHasPersisted) sBootstrapDone = false;
}

void battery_persist_if_due(uint32_t nowMs) {
    if (nowMs - sLastPersistMs < BATT_SAVE_INTERVAL_MS) return;
    sLastPersistMs = nowMs;
    if (!sPrefs.begin(PREF_NS, false)) return;
    sPrefs.putFloat(PREF_KEY_USED_MAH, sTotalMAh);
    sPrefs.end();
}

void battery_update(float currentMA, float voltage, uint32_t dtMs, bool modCurrent) {
    // Bootstrap from OCV if no persisted state
    if (!sHasPersisted && !sBootstrapDone && voltage > 2.5f) {
        float soc = ocvToSoc(voltage);
        sSocPct = constrain(soc, 0.0f, 100.0f);
        sTotalMAh = BATTERY_CAPACITY * (1.0f - sSocPct / 100.0f);
        sShownPct = (int)roundf(sSocPct);
        sBootstrapDone = true;
    }

    // Coulomb counting
    if (modCurrent) {
        float dtHours = dtMs / 3600000.0f;
        sTotalMAh = constrain(sTotalMAh + currentMA * dtHours, 0.0f, BATTERY_CAPACITY);
    }

    // Battery presence debounce
    if (modCurrent) {
        if (voltage <= BATT_PRESENT_OFF_V) sBattLowAccumMs += dtMs; else sBattLowAccumMs = 0;
        if (voltage >= BATT_PRESENT_ON_V)  sBattHighAccumMs += dtMs; else sBattHighAccumMs = 0;
        if (sPresent && sBattLowAccumMs >= BATT_PRESENCE_DEBOUNCE_MS) sPresent = false;
        if (!sPresent && sBattHighAccumMs >= BATT_PRESENCE_DEBOUNCE_MS) sPresent = true;
    }

    // SOC hybrid estimation
    float socFromCount = constrain(100.0f - (sTotalMAh / BATTERY_CAPACITY) * 100.0f, 0.0f, 100.0f);
    bool nearRest = fabsf(currentMA) <= BATT_REST_CURRENT_MA;
    if (nearRest) sRestAccumMs += dtMs; else sRestAccumMs = 0;

    if (modCurrent && sPresent) {
        float socFromOcv = ocvToSoc(voltage);
        float mismatch = fabsf(socFromCount - socFromOcv);
        bool longRest = nearRest && sRestAccumMs >= BATT_REST_TIME_MS;
        float ocvW = 0.05f;
        if (nearRest) ocvW = 0.15f;
        if (longRest) ocvW = 0.35f;
        if (mismatch >= BATT_OCV_MISMATCH_RECOVER_PCT)
            ocvW = max(ocvW, longRest ? 0.50f : 0.25f);
        sSocPct = (1.0f - ocvW) * socFromCount + ocvW * socFromOcv;
        sTotalMAh = constrain(BATTERY_CAPACITY * (1.0f - sSocPct / 100.0f), 0.0f, BATTERY_CAPACITY);
    } else if (modCurrent) {
        sSocPct = 0;
    }

    sSocPct = constrain(sSocPct, 0.0f, 100.0f);

    int target = (int)roundf(sSocPct);
    if (modCurrent && currentMA >= 5.0f && target < sShownPct) sShownPct = target;
    else if (modCurrent && currentMA <= -20.0f && target > sShownPct) sShownPct = target;
    else sShownPct = target;
    sShownPct = constrain(sShownPct, 0, 100);

    float remaining = BATTERY_CAPACITY * (sSocPct / 100.0f);
    sBattLife = (modCurrent && sPresent && fabsf(currentMA) > 0.1f)
                ? (remaining / fabsf(currentMA)) * 60.0f : -1.0f;

    if (modCurrent && !sPresent) {
        sBattLife = -1.0f;
    }
}

float battery_get_percent()  { return (float)sShownPct; }
float battery_get_used_mah() { return sPresent ? sTotalMAh : 0; }
float battery_get_life_min() { return sBattLife; }
bool  battery_is_present()   { return sPresent; }
int   battery_get_shown_pct(){ return sShownPct; }
