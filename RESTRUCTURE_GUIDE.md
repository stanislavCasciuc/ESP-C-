# Ghid de Restructurare — ESP32-C3 Sensor Platform

## Cuprins

- [Problema actuală](#problema-actuală)
- [Structura propusă](#structura-propusă)
- [Modulele și responsabilitățile lor](#modulele-și-responsabilitățile-lor)
- [Best Practices](#best-practices)
- [Librării mai bune — ce să înlocuiești](#librării-mai-bune--ce-să-înlocuiești)
- [Instrucțiuni pentru Agent (AI Refactoring)](#instrucțiuni-pentru-agent-ai-refactoring)
- [Ordinea de execuție](#ordinea-de-execuție)
- [Reguli stricte](#reguli-stricte)

---

## Problema actuală

Întregul firmware (~2900 linii) se află într-un singur fișier `src/main.cpp`. Aceasta cauzează:

- **Imposibilitate de testare** — nu poți testa un modul fără să compilezi tot
- **Conflicte de merge** — orice modificare atinge același fișier
- **Coupling strâns** — variabilele globale sunt accesate de peste tot
- **Timp de onboarding mare** — un contributor nou trebuie să citească tot fișierul
- **Compilare lentă** — orice schimbare recompilează totul
- **Credențiale hardcodate** — WiFi/MQTT passwords direct în cod

---

## Structura propusă

```
ESP-C-/
├── platformio.ini
├── CMakeLists.txt
├── sdkconfig.defaults          # ESP-IDF KConfig overrides
│
├── include/
│   └── config.h                # Toate constantele și pinii (un singur loc)
│
├── src/
│   ├── main.cpp                # setup() + loop() — doar orchestrare, <100 linii
│   │
│   ├── sensors/
│   │   ├── imu.h / imu.cpp            # MPU6500/9250/LSM6DS3 — init, read, fusion
│   │   ├── magnetometer.h / .cpp       # AK8963 — calibrare, heading
│   │   ├── thermistor.h / .cpp         # NTC read + B-coefficient calc
│   │   └── power.h / power.cpp         # INA219 + coulomb counting + SOC
│   │
│   ├── motion/
│   │   ├── fusion.h / fusion.cpp       # Quaternion integration, Kalman, bias estimation
│   │   └── zupt.h / zupt.cpp          # Zero-velocity update, stationary detection
│   │
│   ├── network/
│   │   ├── wifi_manager.h / .cpp       # Multi-SSID connect, AP fallback, NTP sync
│   │   ├── mqtt_client.h / .cpp        # Publish/subscribe, command dispatch
│   │   ├── ws_server.h / .cpp          # WebSocket IMU stream (port 8001)
│   │   └── mqtt_broker.h / .cpp        # sMQTTBroker wrapper (offline mode)
│   │
│   ├── ui/
│   │   ├── display.h / display.cpp     # OLED page rendering
│   │   ├── pages.h / pages.cpp         # Cele 10 pagini (fiecare o funcție)
│   │   └── buttons.h / buttons.cpp     # ISR, debounce, navigare pagini
│   │
│   ├── data/
│   │   ├── json_builder.h / .cpp       # buildJSON(), buildImuJSON(), buildRawGpioJSON()
│   │   └── gpio_monitor.h / .cpp       # GPIO snapshot + event log
│   │
│   ├── system/
│   │   ├── cpu_monitor.h / .cpp        # CPU load measurement, idle hook
│   │   ├── battery.h / battery.cpp     # Battery state persistence (Preferences)
│   │   └── rtc.h / rtc.cpp            # DS1307 init + time sync
│   │
│   └── utils/
│       └── math_helpers.h              # Inline: clamp, lerp, deg2rad, quaternion ops
│
├── data/
│   └── config.json             # Runtime config: WiFi SSIDs, MQTT broker, thresholds
│
├── docs/
│   ├── FRONTEND_MPU9250_HANDOFF.md
│   ├── MQTT_PAYLOADS.md
│   └── ARCHITECTURE.md
│
├── test/
│   ├── test_fusion/
│   │   └── test_quaternion.cpp
│   ├── test_battery/
│   │   └── test_soc.cpp
│   └── test_json/
│       └── test_payloads.cpp
│
└── cloud-server/
    └── ...
```

---

## Modulele și responsabilitățile lor

### `config.h` — Configurare centralizată

```cpp
#pragma once

// ─── Pins ───
constexpr uint8_t PIN_SDA        = 8;
constexpr uint8_t PIN_SCL        = 9;
constexpr uint8_t PIN_BTN_NEXT   = 3;
constexpr uint8_t PIN_BTN_PREV   = 4;
constexpr uint8_t PIN_THERM      = 0;
constexpr uint8_t PIN_IMU_INT    = 10;

// ─── I2C Addresses ───
constexpr uint8_t I2C_OLED       = 0x3C;
constexpr uint8_t I2C_INA219     = 0x40;
constexpr uint8_t I2C_IMU        = 0x68;
constexpr uint8_t I2C_MAG        = 0x0C;

// ─── Battery ───
constexpr float BATTERY_CAPACITY_MAH = 3200.0f;
constexpr float BATTERY_NOMINAL_V    = 3.6f;

// ─── Timing ───
constexpr uint32_t IMU_INTERVAL_MS       = 16;   // ~60 Hz
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;  // 1 Hz
constexpr uint32_t BATTERY_SAVE_INTERVAL = 60000; // 1 min

// ─── Filtering ───
constexpr float GYRO_ALPHA     = 0.25f;
constexpr float KALMAN_Q       = 0.001f;
constexpr float KALMAN_R       = 0.03f;
constexpr float THERM_OFFSET   = -2.0f;
```

### `sensors/imu.h` — Interfață IMU

```cpp
#pragma once
#include <cstdint>

enum class ImuType { NONE, MPU6500, MPU9250, LSM6DS3 };

struct ImuData {
    float ax, ay, az;       // accelerometer (g)
    float gx, gy, gz;       // gyroscope (deg/s)
    float mx, my, mz;       // magnetometer (µT) — zero if unavailable
    bool  mag_present;
};

ImuType imu_init();
bool    imu_read(ImuData& out);
void    imu_set_interrupt(uint8_t pin);
```

### `motion/fusion.h` — Sensor Fusion

```cpp
#pragma once
#include "sensors/imu.h"

struct OrientationState {
    float q[4];             // quaternion wxyz
    float roll, pitch, yaw; // Euler (degrees)
    float heading;          // magnetometer-corrected yaw
    float lin_acc[3];       // gravity-removed, world frame
    float velocity[3];
    float position[3];
    bool  stationary;
};

void fusion_init();
void fusion_update(const ImuData& imu, float dt);
const OrientationState& fusion_get_state();
```

### `network/wifi_manager.h` — WiFi cu fallback

```cpp
#pragma once

enum class NetMode { DISCONNECTED, STATION, SOFTAP };

void     wifi_init();        // reads config.json for SSIDs
NetMode  wifi_get_mode();
int      wifi_rssi();
bool     wifi_try_connect(); // attempts saved SSIDs, falls back to AP
void     wifi_skip();        // button-triggered skip during connect
```

### `main.cpp` — Doar orchestrare

```cpp
#include "config.h"
#include "sensors/imu.h"
#include "sensors/thermistor.h"
#include "sensors/power.h"
#include "motion/fusion.h"
#include "network/wifi_manager.h"
#include "network/mqtt_client.h"
#include "network/ws_server.h"
#include "ui/display.h"
#include "ui/buttons.h"
#include "system/cpu_monitor.h"
#include "system/battery.h"
#include "data/json_builder.h"

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);

    display_init();
    buttons_init();
    power_init();
    battery_load_state();
    imu_init();
    fusion_init();
    thermistor_init();
    cpu_monitor_init();
    wifi_init();
    mqtt_init();
    ws_init();
}

void loop() {
    uint32_t now = millis();

    // 60 Hz — IMU + fusion + WebSocket stream
    if (now - last_imu >= IMU_INTERVAL_MS) {
        ImuData imu;
        if (imu_read(imu)) {
            fusion_update(imu, (now - last_imu) / 1000.0f);
            ws_broadcast_imu(fusion_get_state());
        }
        last_imu = now;
    }

    // 1 Hz — sensors + MQTT + display
    if (now - last_telem >= TELEMETRY_INTERVAL_MS) {
        mqtt_publish_telemetry();
        mqtt_publish_raw_gpio();
        display_update();
        last_telem = now;
    }

    mqtt_loop();
    ws_loop();
    buttons_poll();
    battery_periodic_save(now);
}
```

---

## Best Practices

### 1. Un modul = un header + un source

Fiecare `.h` expune **doar interfața publică**. Variabilele de stare sunt `static` în `.cpp`.

```
❌  extern float gx, gy, gz;          // global în main.cpp
✅  bool imu_read(ImuData& out);       // struct cu date, acces prin funcție
```

### 2. Datele circulă prin structuri, nu prin variabile globale

```
❌  float q0, q1, q2, q3;            // 4 globale separate
✅  struct OrientationState { float q[4]; ... };
```

### 3. Dependențe unidirecționale

```
config.h ← sensors/ ← motion/ ← data/json_builder
                ↑                      ↑
             system/              network/
                                     ↑
                                    ui/
```

Niciun modul nu importă din `main.cpp`. Fluxul de date merge de la senzori spre rețea/UI, nu invers.

### 5. ISR-uri minime

ISR-urile setează doar un flag atomic. Logica se rulează în `loop()`:

```cpp
// buttons.cpp
volatile bool btn_next_pressed = false;

void IRAM_ATTR onBtnNext() { btn_next_pressed = true; }

void buttons_poll() {
    if (btn_next_pressed) {
        btn_next_pressed = false;
        display_next_page();
    }
}
```

### 6. Fără `delay()` în loop

Totul se face cu timere millis-based (deja implementat, de menținut).

### 7. Fiecare pagină OLED = o funcție separată

```cpp
// pages.cpp
typedef void (*PageRenderer)(Adafruit_SSD1306& oled);

static const PageRenderer pages[] = {
    page_dashboard,
    page_temperature,
    page_gyroscope,
    page_wifi_status,
    page_cpu_load,
    page_voltage,
    page_current,
    page_battery,
    page_gpio_log,
    page_i2c_scanner,
};
```

### 7. Testabilitate

Modulele `fusion`, `battery/soc`, `json_builder` nu depind de hardware. Se pot testa nativ:

```ini
; platformio.ini
[env:native_test]
platform = native
test_build_src = yes
test_filter = test_fusion, test_battery, test_json
```

### 8. Compilare incrementală

Cu fișiere separate, PlatformIO recompilează doar modulul modificat. Pe un proiect de 2900 linii asta reduce build time de la ~30s la ~5s pentru schimbări mici.

### 9. Documentație lângă cod

Fiecare modul `.h` are un comentariu de 2-3 linii care explică scopul. Nu docstrings excesive, doar ce este non-obvious.

---

## Librării mai bune — ce să înlocuiești

Proiectul actual scrie **mult cod manual** acolo unde există librării testate și optimizate. Mai jos e o analiză a fiecărui component cu alternativa recomandată.

### 1. Sensor Fusion — `updateOrientationFusion()` scris de la zero

**Problema**: ~200 linii de quaternion integration manual + Kalman custom + yaw mag correction + bias estimation. Codul funcționează dar:
- Kalman-ul e simplificat (2-state per axă, fără cross-coupling)
- Complementary filter-ul quaternion e un Mahony nedeclarat, cu gain-uri hardcodate
- Nu suportă auto-tuning sau adaptive gains
- Orice modificare necesită recalibrare manuală pe hardware

**Înlocuiește cu**: [**Fusion (by x-io Technologies)**](https://github.com/xioTechnologies/Fusion)

```ini
; platformio.ini
lib_deps =
    https://github.com/xioTechnologies/Fusion.git
```

```cpp
#include "Fusion.h"

FusionAhrs ahrs;
FusionOffset offset;

void fusion_init() {
    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, IMU_SAMPLE_RATE);  // e.g. 60

    // Tuning — aceste valori înlocuiesc QUAT_ACC_GAIN, KALMAN_Q_*, etc.
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,                    // accelerometer correction strength
        .gyroscopeRange = 500.0f,        // ±500 dps (ca MPU config actual)
        .accelerationRejection = 10.0f,  // ignoră accel când != 1g
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * IMU_SAMPLE_RATE,
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}

void fusion_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz, float dt) {
    FusionVector gyro = {gx, gy, gz};
    FusionVector accel = {ax, ay, az};
    FusionVector mag = {mx, my, mz};

    gyro = FusionOffsetUpdate(&offset, gyro);  // auto bias removal!

    if (mag_present)
        FusionAhrsUpdate(&ahrs, gyro, accel, mag, dt);
    else
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, dt);
}

// Rezultate:
FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
FusionEuler euler = FusionQuaternionToEuler(q);
FusionVector lin = FusionAhrsGetLinearAcceleration(&ahrs); // gravity-free!
```

**Ce elimini**: `updateOrientationFusion()`, `kalmanUpdateAxis()`, `KalmanAxis`, `updateEulerFromQuaternion()`, `updateInertialBiases()`, toți parametrii Kalman. ~300 linii de cod → ~30 linii.

### 2. IMU Driver — registre I2C scrise manual

**Problema**: ~150 linii de `Wire.beginTransmission()` / `Wire.write()` pentru MPU6500/9250 init, config registers, burst read, magnetometer bypass. Plus fallback la LSM6DS3 tot manual.

**Înlocuiește cu**: Librării dedicate care gestionează registrele intern:

| Senzor | Librărie recomandată | PlatformIO |
|--------|---------------------|------------|
| MPU6500/9250 | [**Bolder Flight MPU9250**](https://github.com/bolderflight/mpu9250) | `bolderflight/Bolder Flight Systems MPU9250` |
| LSM6DS3 | [**SparkFun LSM6DS3**](https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library) | `sparkfun/SparkFun LSM6DS3 Breakout` |
| AK8963 (mag) | Inclusă în Bolder Flight MPU9250 | — |

```cpp
#include "MPU9250.h"

MPU9250 imu(Wire, 0x68);

void imu_init() {
    imu.begin();
    imu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
    imu.setSrd(0);  // 1 kHz output
    imu.enableDataReadyInterrupt();  // pe GYRO_INT pin
}

bool imu_read(ImuData& out) {
    if (imu.readSensor() < 0) return false;
    out.ax = imu.getAccelX_mss() / 9.80665f;  // m/s² → g
    out.ay = imu.getAccelY_mss() / 9.80665f;
    out.az = imu.getAccelZ_mss() / 9.80665f;
    out.gx = imu.getGyroX_rads() * 57.29578f; // rad/s → deg/s
    out.gy = imu.getGyroY_rads() * 57.29578f;
    out.gz = imu.getGyroZ_rads() * 57.29578f;
    out.mx = imu.getMagX_uT();  // deja calibrat intern
    out.my = imu.getMagY_uT();
    out.mz = imu.getMagZ_uT();
    out.mag_present = true;
    return true;
}
```

**Ce elimini**: toate constantele `MPU_*`, `AK8963_*`, `LSM_*`, funcțiile `i2cWriteReg()`, `i2cReadBytes()`, `initGyro()`, `readGyro()`, `readMagnetometer()`, bypass mode setup. ~200 linii → ~25 linii.

### 3. Thermistor — calcul manual B-coefficient

**Problema**: Librăria `sbe3binkz/Thermistor` din `platformio.ini` există dar codul actual face totul manual cu `thermistorTempFromRatio()` și `thermistorTempFromResistance()` (~40 linii).

**Opțiune A** — Folosește librăria deja inclusă dar corect:
```cpp
Thermistor therm(10000, 10000, 3950, 12, 3.3, 298.15);
float temp = therm.getTempC();  // face intern tot calculul
```

**Opțiune B** — [**NTC_Thermistor**](https://github.com/YuriiSalimov/NTC_Thermistor) (mai populară, mai testată):
```ini
lib_deps = yurii-salimov/NTC_Thermistor
```
```cpp
#include <NTC_Thermistor.h>
NTC_Thermistor therm(THERM_PIN, 10000, 10000, 25, 3950);
float temp = therm.readCelsius();  // + offset calibrare
```

### 4. Battery SOC — OCV table + coulomb counting manual

**Problema**: `ocvToSocPct()` e o lookup table manuală cu interpolație liniară (~30 linii). Coulomb counting e manual cu `gTotalMAh += current * dt`. Funcționează, dar:
- Tabelul OCV e generic, nu specific pentru LG INR18650MH1
- Nu compensează temperatura
- Coulomb counting driftează fără periodic OCV correction

**Alternative**:

Nu există o librărie Arduino drop-in perfectă pentru battery SOC, dar se poate simplifica:

```cpp
// Lookup table specific pentru LG MH1 (din datasheet)
static const float OCV_TABLE[][2] = {
    {4.20, 100}, {4.10, 87}, {4.00, 72}, {3.90, 57},
    {3.80, 43},  {3.70, 28}, {3.60, 17}, {3.50, 9},
    {3.40, 4},   {3.30, 1},  {3.00, 0}
};
```

Recomandare: păstrează logica actuală dar mută-o într-un modul `battery.cpp` curat. SOC estimation e un domeniu unde custom code are sens — librăriile generice nu știu specificul bateriei tale.

### 5. ZUPT / Position Estimation — de la zero

**Problema**: Integrarea de viteză/poziție din accelerometru (~80 linii) driftează inevitabil. ZUPT-ul ajută dar nu elimină drift-ul pe termen lung.

**Recomandare**: Dacă se folosește ca input VR/3D (cf. `FRONTEND_MPU9250_HANDOFF.md`), consideră:
- **Doar quaternion + Euler** — fără position tracking (drift-ul face datele inutile după ~10s)
- **Fusion library** are built-in `FusionAhrsGetLinearAcceleration()` care elimină gravitatea corect
- Position estimation (dacă e necesar) se mută într-un modul separat `motion/position.cpp` cu disclaimer clar

### 6. CPU Load — idle hook custom

**Problema**: ~80 linii de idle hook + exponential filter + runtime stats fallback. Funcționează, dar ESP-IDF are API nativ:

```cpp
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

float getCpuLoad() {
    // ESP-IDF native — mult mai precis
    TaskStatus_t *taskStatusArray;
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    uint32_t totalRunTime;
    taskStatusArray = (TaskStatus_t*)pvPortMalloc(taskCount * sizeof(TaskStatus_t));
    uxTaskGetSystemState(taskStatusArray, taskCount, &totalRunTime);
    // ... calcul din runtime counters
    vPortFree(taskStatusArray);
}
```

Sau mai simplu, folosind [`esp_freertos_hooks.h`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/system/freertos_additions.html) care e deja importat — dar codul actual duplică logica intern. Se poate reduce la ~20 linii.

### 7. WiFi Manager — manual multi-SSID loop

**Problema**: ~60 linii de WiFi connect loop cu timeout, button skip, AP fallback. Funcțional dar fragil (blocking delay, hardcoded timeouts).

**Înlocuiește cu**: [**WiFiManager**](https://github.com/tzapu/WiFiManager) sau [**ESPAsync_WiFiManager**](https://github.com/khoih-prog/ESPAsync_WiFiManager)

```ini
lib_deps = https://github.com/tzapu/WiFiManager.git
```

```cpp
#include <WiFiManager.h>
WiFiManager wm;

void wifi_init() {
    wm.setConfigPortalTimeout(180);  // 3 min captive portal
    if (!wm.autoConnect("HS-ESP32", "esp32hs123")) {
        // fallback: rămâne AP mode
        startOfflineServices();
    }
}
```

**Beneficii**: captive portal cu web UI, salvare SSID/pass în NVS automat, fallback AP built-in, zero credentials hardcodate.

### 8. SparkFun MPU9250-DMP — problematic pe ESP32-C3

**Problema**: Codul are `#define USE_MPU_DMP 0` cu comentariul *"DMP path can hang on some ESP32-C3 + MPU setups during boot"*. Librăria SparkFun e deprecated și are probleme de compatibilitate.

**Înlocuiește cu**: Bolder Flight MPU9250 (menționat la punctul 2) + Fusion AHRS (punctul 1). Combinația acestor două înlocuiește complet nevoia de DMP on-chip.

### Sumar: Impact librării noi

| Zona | Cod actual (linii) | Cu librării (linii) | Librărie |
|------|--------------------|--------------------|----------|
| Sensor fusion | ~300 | ~30 | Fusion (x-io) |
| IMU driver | ~200 | ~25 | Bolder Flight MPU9250 |
| Magnetometer | ~60 | 0 (inclusă în IMU lib) | — |
| Thermistor | ~40 | ~5 | NTC_Thermistor |
| WiFi connect | ~60 | ~10 | WiFiManager |
| CPU load | ~80 | ~20 | ESP-IDF native |
| **Total** | **~740** | **~90** | |

**Reducere estimată**: ~650 linii de cod custom → librării testate. Plus eliminarea a ~20 de constante de tuning (Kalman Q/R, gains, alpha-uri) care devin parametri interni ai librăriilor.

### platformio.ini actualizat

```ini
lib_deps = 
    adafruit/Adafruit GFX Library
    adafruit/Adafruit SSD1306
    adafruit/Adafruit INA219
    adafruit/RTClib
    bolderflight/Bolder Flight Systems MPU9250
    sparkfun/SparkFun LSM6DS3 Breakout
    https://github.com/xioTechnologies/Fusion.git
    links2004/WebSockets
    knolleary/PubSubClient
    bblanchon/ArduinoJson
    terrorsl/sMQTTBroker
    ricmoo/QRCode
    yurii-salimov/NTC_Thermistor
    https://github.com/tzapu/WiFiManager.git
```

**Eliminat**: `SparkFun MPU-9250-DMP` (problematic), `sbe3binkz/Thermistor` (înlocuit cu NTC_Thermistor).  
**Adăugat**: Fusion, Bolder Flight MPU9250, SparkFun LSM6DS3, NTC_Thermistor, WiFiManager.

---

## Instrucțiuni pentru Agent (AI Refactoring)

### Obiectiv

Să spargi `main.cpp` (~2900 linii) în modulele definite mai sus, **fără să schimbi comportamentul**. Refactorizare pură — zero features noi.

### Ordinea de execuție

Agentul trebuie să urmeze această ordine **strict secvențială**:

| Pas | Acțiune | Verificare |
|-----|---------|------------|
| 1 | **Creează `config.h`** — extrage TOATE constantele, pinii, adresele I2C din top-ul `main.cpp` | Compilează fără erori |
| 2 | **Extrage `sensors/imu.cpp`** — tot codul MPU6500/9250/LSM6DS3: init, register read/write, data-ready ISR | `imu_init()` + `imu_read()` funcționează |
| 3 | **Extrage `sensors/magnetometer.cpp`** — AK8963 init, read, calibrare | Heading se calculează corect |
| 4 | **Extrage `sensors/thermistor.cpp`** — analogRead + formula B-coefficient | Temperatura citită corect |
| 5 | **Extrage `sensors/power.cpp`** — INA219 init, voltage/current/power read | Valori identice cu originalul |
| 6 | **Extrage `motion/fusion.cpp`** — quaternion integration, Kalman, gravity separation, linear accel, velocity, position, ZUPT | Quaternionul și Euler angles identice |
| 7 | **Extrage `network/wifi_manager.cpp`** — WiFi multi-SSID, AP fallback, NTP | Se conectează la rețea |
| 8 | **Extrage `network/mqtt_client.cpp`** — PubSubClient wrapper, publish/subscribe, command handler | MQTT funcțional |
| 9 | **Extrage `network/ws_server.cpp`** — WebSocket broadcast, client tracking | Stream 60 Hz funcțional |
| 10 | **Extrage `network/mqtt_broker.cpp`** — sMQTTBroker wrapper offline | Offline mode funcțional |
| 11 | **Extrage `ui/display.cpp` + `ui/pages.cpp` + `ui/buttons.cpp`** — OLED rendering, pagini, butoane | Display funcțional, navigare OK |
| 12 | **Extrage `data/json_builder.cpp`** — cele 3 funcții JSON | Payload-urile identice |
| 13 | **Extrage `data/gpio_monitor.cpp`** — GPIO snapshot + event log | GPIO raw payload OK |
| 14 | **Extrage `system/cpu_monitor.cpp`** — idle hook, stats, exponential filter | CPU% identic |
| 15 | **Extrage `system/battery.cpp`** — Preferences save/load, coulomb counting, SOC hybrid | Battery state persistent |
| 16 | **Extrage `system/rtc.cpp`** — DS1307 init + time read | RTC funcțional |
| 17 | **Rescrie `main.cpp`** — doar include-uri + `setup()` + `loop()` (~80-100 linii) | Compilează, toate features funcționează |
| 18 | **Mută docs** — `FRONTEND_MPU9250_HANDOFF.md` și `MQTT_PAYLOADS.md` în `docs/` | — |
| 19 | **Creează `data/config.example.json`** — template fără credențiale reale | — |
| 20 | **Actualizează `.gitignore`** — adaugă `data/config.json` | — |

### Reguli stricte

1. **Un pas = o compilare reușită.**  
   După fiecare extragere, proiectul TREBUIE să compileze cu `pio run`. Dacă nu compilează, fixează înainte de a trece la pasul următor.

2. **Nu schimba logica.**  
   Dacă originalul are un bug sau un workaround, menține-l exact. Refactorizarea nu este momentul pentru bugfix-uri.

3. **Dependențe noi doar cele din secțiunea "Librării mai bune".**  
   Nu adăuga alte librării în afara celor recomandate în acest ghid.

4. **Păstrează ordinea de inițializare.**  
   Unele module depind de `Wire.begin()` sau WiFi fiind activ. Respectă secvența din `setup()` original.

5. **Variabilele globale partajate devin parametri sau getters.**  
   Exemplu: `mqtt_client.cpp` nu accesează direct `temperature` — primește valoarea prin `buildJSON()` care apelează `thermistor_read()`.

6. **Fișierele `.h` au include guards (`#pragma once`).**

7. **Fiecare `.cpp` include doar ce folosește** — nu include tranzitiv prin alte headere.

8. **Testează pe hardware real** după pașii 6, 11 și 17 (fusion, display, final).

9. **Commit la fiecare pas reușit** cu mesaj descriptiv:
   ```
   refactor: extract sensors/imu module from main.cpp
   ```

10. **Nu șterge `main.cpp` original** până la pasul final. Lucrează într-un branch `refactor/modularize`.

### Strategia de extragere (pentru fiecare modul)

```
1. Identifică blocul de cod din main.cpp (funcții + variabile + includes)
2. Creează .h cu interfața publică (structs, funcții, constante exportate)
3. Creează .cpp cu implementarea (variabile static, funcții interne static)
4. În main.cpp: înlocuiește blocul extras cu #include "modul.h" + apeluri
5. Compilează: `pio run -e esp32c3`
6. Rezolvă erorile (de obicei: missing include, undefined variable, wrong scope)
7. Compilează din nou → succes → commit → next
```

### Ce să NU facă agentul

- ❌ Nu crea clase C++ cu moștenire / polimorfism — firmware-ul e C-style, menține-l așa
- ❌ Nu adăuga `namespace`-uri — namespace-urile pe ESP32 cu Arduino adaugă complexitate fără beneficiu
- ❌ Nu inventa algoritmi proprii de fusion — folosește librăria Fusion (x-io)
- ❌ Nu optimiza prematur (inline, `__attribute__`, pragma pack) — compilatorul se descurcă
- ❌ Nu muta logica între module — dacă `buildJSON()` citește direct `gx`, refactorizează doar accesul, nu muta calculul
- ❌ Nu crea fișiere `.ino` — proiectul e PlatformIO, nu Arduino IDE
- ❌ Nu adăuga logging library — `Serial.println` e suficient pentru debug

---

## Metrici de succes

După restructurare, verifică:

| Criteriu | Țintă |
|----------|-------|
| `main.cpp` linii | < 120 |
| Cel mai mare fișier | < 400 linii |
| Compilare incrementală (1 fișier schimbat) | < 8s |
| Toate features funcționale | identic cu originalul |
| Cod custom înlocuit cu librării | fusion, IMU driver, thermistor, WiFi |
| Teste native (fusion, SOC, JSON) | pass |
| `pio run` clean build | 0 warnings (sau identice cu originalul) |
