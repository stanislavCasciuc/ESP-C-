#include "cpu_monitor.h"
#include "../network/mqtt_client.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <cmath>

static volatile uint32_t sIdleHookCount = 0;
static uint32_t sPrevIdleCount = 0;
static uint32_t sLastSampleMs = 0;
static float sIdleRateRef = 0.0f;
static float sFiltered = 0.0f;
static bool sFilterSeeded = false;
static float sLoad = 0.0f;

// Runtime stats
static uint32_t sPrevTotalRunTime = 0;
static uint32_t sPrevIdleRunTime = 0;
static bool sRuntimeSeeded = false;

static bool idleHook() { sIdleHookCount++; return true; }

static bool getLoadFromRuntimeStats(float& out) {
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    if (taskCount == 0) return false;
    TaskStatus_t* stats = (TaskStatus_t*)pvPortMalloc(taskCount * sizeof(TaskStatus_t));
    if (!stats) return false;
    uint32_t totalRunTime = 0;
    UBaseType_t filled = uxTaskGetSystemState(stats, taskCount, &totalRunTime);
    uint32_t idleRunTime = 0;
    for (UBaseType_t i = 0; i < filled; i++) {
        if (strncmp(stats[i].pcTaskName, "IDLE", 4) == 0)
            idleRunTime += stats[i].ulRunTimeCounter;
    }
    vPortFree(stats);
    if (!sRuntimeSeeded) {
        sRuntimeSeeded = true;
        sPrevTotalRunTime = totalRunTime;
        sPrevIdleRunTime = idleRunTime;
        return false;
    }
    uint32_t dT = totalRunTime - sPrevTotalRunTime;
    uint32_t dI = idleRunTime - sPrevIdleRunTime;
    sPrevTotalRunTime = totalRunTime;
    sPrevIdleRunTime = idleRunTime;
    if (dT == 0) return false;
    out = constrain(100.0f - (100.0f * (float)dI / (float)dT), 0.0f, 100.0f);
    return true;
#else
    (void)out;
    return false;
#endif
}

static void cpuStressTask(void* parm) {
    (void)parm;
    volatile float sink = 0.0001f;
    for (;;) {
        if (mqtt_get_module_cpu_stress()) {
            for (int i = 1; i <= 800; i++) {
                float x = (float)i + sink;
                sink += sqrtf(x) * logf(x + 1.0f);
                sink = fmodf(sink, 1000.0f);
            }
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

void cpu_init() {
    bool ok = false;
#if CONFIG_FREERTOS_NUMBER_OF_CORES == 1
    ok = esp_register_freertos_idle_hook(idleHook);
#else
    ok = esp_register_freertos_idle_hook_for_cpu(idleHook, 0);
#endif
    if (!ok) Serial.println("[CPU] Idle hook not available");
    sPrevIdleCount = sIdleHookCount;
    sLastSampleMs = millis();
}

void cpu_start_stress_task() {
    xTaskCreate(cpuStressTask, "cpu_stress", 3072, nullptr, 1, nullptr);
}

float cpu_get_load() {
    uint32_t nowMs = millis();
    if (sLastSampleMs == 0) { sLastSampleMs = nowMs; sPrevIdleCount = sIdleHookCount; }

    uint32_t elapsed = nowMs - sLastSampleMs;
    if (elapsed < 1000U) return sLoad;

    float loadOut = 0;
    bool measured = false;
    if (getLoadFromRuntimeStats(loadOut)) {
        measured = true;
    } else {
        uint32_t idleNow = sIdleHookCount;
        uint32_t idleDelta = idleNow - sPrevIdleCount;
        sPrevIdleCount = idleNow;
        float idleRate = (elapsed > 0) ? ((float)idleDelta / (float)elapsed) : 0;
        if (sIdleRateRef < 0.001f) {
            sIdleRateRef = idleRate; loadOut = 0; measured = true;
        } else {
            if (idleRate > sIdleRateRef) sIdleRateRef = idleRate;
            else sIdleRateRef = sIdleRateRef * 0.999f + idleRate * 0.001f;
            if (sIdleRateRef < 0.001f) loadOut = 0;
            else loadOut = constrain(100.0f - (100.0f * idleRate / sIdleRateRef), 0.0f, 100.0f);
            measured = true;
        }
    }

    sLastSampleMs = nowMs;
    if (measured) {
        float target = constrain(loadOut, 0.0f, 100.0f);
        if (!sFilterSeeded) { sFiltered = target; sFilterSeeded = true; }
        else {
            float alpha = (target > sFiltered) ? 0.35f : 0.20f;
            float step = sFiltered + alpha * (target - sFiltered) - sFiltered;
            if (step > 3.0f) step = 3.0f;
            if (step < -3.0f) step = -3.0f;
            sFiltered += step;
        }
        if (sFiltered < 0.3f) sFiltered = 0.0f;
        sLoad = constrain(sFiltered, 0.0f, 100.0f);
    }
    return sLoad;
}
