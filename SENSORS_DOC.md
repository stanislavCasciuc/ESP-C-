# Documentație Senzori & Module — ESP32-C3 Super Mini

> Generată după restructurarea modulară a proiectului.  
> Fiecare secțiune descrie **ce face** modulul, **ce librărie/formulă** folosește, **exemple de date**, **ce este scris dar nefolosit** și **ce s-ar putea îmbunătăți**.

---

## Cuprins

1. [Termistor NTC 10 kΩ](#1-termistor-ntc-10-kΩ)
2. [IMU — MPU6500/9250 + LSM6DS3 (fallback)](#2-imu--mpu65009250--lsm6ds3-fallback)
3. [Magnetometru AK8963](#3-magnetometru-ak8963)
4. [Senzor de putere INA219](#4-senzor-de-putere-ina219)
5. [Fusion — Quaternion AHRS](#5-fusion--quaternion-ahrs)
6. [Estimare mișcare (poziție/viteză)](#6-estimare-mișcare-pozițieviteză)
7. [Baterie — Coulomb counting + OCV hibrid](#7-baterie--coulomb-counting--ocv-hibrid)
8. [RTC — DS1307](#8-rtc--ds1307)
9. [Monitor CPU](#9-monitor-cpu)
10. [Monitor GPIO](#10-monitor-gpio)
11. [Display OLED SSD1306](#11-display-oled-ssd1306)
12. [Constante definite dar nefolosite în codul nou](#12-constante-definite-dar-nefolosite-în-codul-nou)
13. [Rezumat îmbunătățiri posibile](#13-rezumat-îmbunătățiri-posibile)

---

## 1. Termistor NTC 10 kΩ

**Fișiere:** `src/sensors/thermistor.cpp`, `src/sensors/thermistor.h`

### Cum funcționează

Termistorul NTC este conectat într-un **divizor de tensiune** cu o rezistență fixă de 10 kΩ, citit prin ADC-ul ESP32-C3 (12 biți, 0–4095).

### Librărie

Nu folosește librărie externă — implementare manuală cu formule fizice.

### Formule

1. **Citire ADC** — media a 8 eșantioane, cu pauze de 250 µs între ele:
   ```
   sRaw = (adc0 + adc1 + ... + adc7) / 8
   ```

2. **Tensiunea ADC:**
   ```
   V_adc = (sRaw / 4095.0) × 3.3V
   ```

3. **Rezistența NTC din divizor de tensiune:**
   ```
   R_ntc = R_fixă × (V_adc / (V_supply − V_adc))
   ```
   unde `V_supply` este luat de la INA219 (dacă e disponibil), altfel fallback la 4.0V.

4. **Ecuația Steinhart–Hart simplificată (coeficient B):**
   ```
   1/T = (1/T₀) + (1/B) × ln(R_ntc / R₂₅)
   ```
   - `T₀` = 25°C + 273.15 = 298.15 K
   - `B` = 3950
   - `R₂₅` = 10 000 Ω (rezistența la 25°C)

5. **Calibrare post:**
   ```
   T_final = T × THERM_CAL_GAIN + THERM_CAL_OFFSET_C
   ```
   Actual: gain = 1.0, offset = −2.0°C

6. **Filtru EMA (Exponential Moving Average):**
   ```
   filtered = filtered + α × (T_final − filtered)
   ```
   `α = 0.35` (THERM_FILTER_ALPHA)

### Exemplu de date JSON

```json
{ "temp": 24.3, "ntc_c": 24.3, "ntc_raw": 2187, "temp_raw": 2187 }
```

### Protecții
- Valoarea respinsă dacă ADC ≤ 1 sau ≥ 4094
- Rezistență invalidă dacă < 100 Ω sau > 1 MΩ
- Temperatură respinsă dacă < −40°C sau > 125°C
- Salt brusc > 20°C față de valoarea filtrată → ignorat (spike rejection)

### Nefolosit
- **`THERM_NTC_TO_GND`** — definit în `config.h` dar **niciodată verificat** în codul nou; formula presupune mereu NTC-to-VCC.

### Îmbunătățiri posibile
- Implementare a configurării `THERM_NTC_TO_GND` (formula de calcul `R_ntc` diferă dacă NTC e conectat la GND)
- Calibrare multi-punct (3 coeficienți Steinhart–Hart complet) în loc de ecuația B simplificată
- Supraesantionare ADC (oversampling) direct cu funcția ESP32 `analogReadMilliVolts()` pentru precizie mai bună

---

## 2. IMU — MPU6500/9250 + LSM6DS3 (fallback)

**Fișiere:** `src/sensors/imu.cpp`, `src/sensors/imu.h`

### Cum funcționează

La inițializare se încearcă în ordine:
1. **MPU6500/9250** la adresele I2C `0x69` apoi `0x68` (verificare WHO_AM_I = 0x70 sau 0x71)
2. Dacă nu se găsește → **LSM6DS3** la adresa `0x6A` (WHO_AM_I = 0x69)

### Librărie

**Fără librărie externă** — comunicare I2C directă prin `Wire.h`. Totul este implementat manual: configurare registre, citire raw, calibrare, scalare.

### Configurare MPU

| Registru | Valoare | Descriere |
|----------|---------|-----------|
| PWR_MGMT_1 | 0x01 | Clock = PLL cu giroscop X |
| CONFIG | 0x04 | DLPF ~20 Hz |
| SMPLRT_DIV | 0x00 | Sample rate = 1 kHz |
| GYRO_CONFIG | 0x08 | ±500 dps |
| ACCEL_CONFIG | 0x00 | ±2g |
| INT_PIN_CFG | 0x02 | I2C bypass (pentru AK8963) |
| INT_ENABLE | 0x01 | Data-ready interrupt |

### Formule de scalare

**MPU Gyroscope (±500 dps):**
```
gyro_dps = raw × (1 / 65.5)    // MPU_GYRO_SCALE_DPS
```

**MPU Accelerometer (±2g):**
```
accel_g = raw × (1 / 16384.0)  // MPU_ACCEL_SCALE_G
```

**LSM6DS3 Gyroscope (±2000 dps):**
```
gyro_dps = raw × 0.07           // LSM_GYRO_SCALE_DPS
```

### Calibrare giroscop (la boot)

La inițializare se colectează 96 de eșantioane (MPU_BIAS_SAMPLES) cu ESP-ul imobil:
```
gyroBias_X = Σ(raw_X) / 96
gyroBias_Y = Σ(raw_Y) / 96
gyroBias_Z = Σ(raw_Z) / 96
```
Bias-ul este apoi scăzut din fiecare citire.

### Recalibrare runtime (adaptivă)

Când dispozitivul este imobil (detectat de modulul fusion), bias-urile se actualizează lent:
```
gyroBias += α × (gyroRaw − gyroBias)
accelBias += α × ((accelMeas − gravity) − accelBias)
```
- `α_gyro_still` = 0.015, `α_gyro_recal` = 0.06
- `α_accel_still` = 0.01, `α_accel_recal` = 0.04
- Interval minim: 300 secunde (BIAS_RECAL_INTERVAL_MS)

### Filtru giroscop (EMA + deadband)

```
filtered += 0.25 × (raw − filtered)     // GYRO_FILTER_ALPHA
dacă |filtered| < 0.5 → filtered = 0    // GYRO_DEADBAND_DPS
```

### ISR Data-Ready

Pe MPU, pinul GPIO10 este setat ca interrupt RISING → `imu_isr_data_ready()` incrementează un contor atomic, consumat în `loop()` cu maxim 3 citiri per ciclu.

### Exemplu de date JSON

```json
{
  "gx": -0.3, "gy": 1.2, "gz": 0.0,
  "gabs": 1.2,
  "ax": 0.012, "ay": -0.003, "az": 0.998,
  "imu_addr": 105,
  "mag_present": true
}
```

### Nefolosit
- **DMP (Digital Motion Processor)** — flag-ul `sMpuDmpActive` / `imu_is_dmp_active()` este expus dar mereu `false`; codul DMP din `main.cpp` (vechi) nu a fost portat

### Îmbunătățiri posibile
- Activarea DMP-ului MPU pentru orientare hardware (reduce încărcarea CPU)
- Citirea accelerometrului pe LSM6DS3 (actualmente returnează mereu 0)
- Self-test IMU la boot (registrele de self-test sunt disponibile pe MPU)
- Calibrare magnetometru (hard/soft iron) — actualmente se folosesc doar coeficienții ASA din fabrică

---

## 3. Magnetometru AK8963

**Fișiere:** `src/sensors/imu.cpp` (integrat în modulul IMU)

### Cum funcționează

AK8963 este magnetometrul integrat în MPU-9250, accesibil prin I2C bypass (adresa `0x0C`). Este detectat automat dacă MPU-ul este găsit.

### Configurare

1. Citire coeficienți de sensibilitate (ASA) din registrele de calibrare din fabrică (ROM mode `0x0F`)
2. Setare mod continuu 16-bit la 100 Hz (`CNTL1 = 0x16`)

### Formule

**Ajustare sensibilitate (coeficienți fabrică):**
```
MagAdj_X = (ASA_X − 128) / 256 + 1.0
```

**Conversie la µT (microtesla):**
```
mag_uT = rawValue × 0.15 × MagAdj
```
(0.15 µT/LSB la rezoluție 16-bit)

### Protecții
- Se verifică overflow bit (ST2 registru, bit 3)
- Se citește doar dacă data-ready (ST1 bit 0)
- Interval minim de 10 ms între citiri

### Nefolosit
- **Calibrarea hard-iron / soft-iron** — nu se face; valorile brute sunt trimise direct la fusion

### Îmbunătățiri posibile
- Implementare calibrare hard-iron (offset) și soft-iron (scalare per axă) — ellipsoid fitting
- Afișare heading pe OLED (busolă digitală)

---

## 4. Senzor de putere INA219

**Fișiere:** `src/sensors/power.cpp`, `src/sensors/power.h`

### Cum funcționează

INA219 măsoară tensiunea bus și curentul printr-un shunt de precizie. Este conectat la adresa I2C `0x40`.

### Librărie

**Adafruit_INA219** (`Adafruit INA219 @ 1.2.3`)

### Configurare

Calibrare: `setCalibration_16V_400mA()` — optimizat pentru măsurare de curent scăzut (rezoluție maximă).

### Ce citește

| Valoare | Funcție | Registru raw |
|---------|---------|--------------|
| Tensiune bus (V) | `getBusVoltage_V()` | 0x02 |
| Curent (mA) | `getCurrent_mA()` | 0x04 |
| Putere (mW) | `getPower_mW()` | 0x03 |

### Protecții
- Valorile NaN/Inf sunt înlocuite cu 0
- Deadband curent: dacă `|current| < 5 mA` → curent = 0 (elimină zgomotul de repaus)

### Exemplu de date JSON

```json
{
  "v": 3.87,
  "ma": 42.3,
  "mw": 164
}
```

### Nefolosit
- **Registrele raw** (`power_get_bus_raw()`, `power_get_current_raw()`, `power_get_power_raw()`) sunt citite și expuse în API dar nu apar în JSON-ul MQTT transmis; se folosesc doar pe pagina GPIO log a OLED-ului

### Îmbunătățiri posibile
- Alert pe supracurent (INA219 are registru de alertă hardware)
- Mod continuu de eșantionare cu mediere hardware pentru precizie mai bună
- Publicare raw registers prin MQTT pentru diagnoză

---

## 5. Fusion — Quaternion AHRS

**Fișiere:** `src/motion/fusion.cpp`, `src/motion/fusion.h`

### Cum funcționează

Algoritm de tip **Complementary Filter pe quaternioni** (asemănător Mahony), nu folosește Kalman complet.

### Librărie

**Fără librărie** — implementat manual.

### Algoritmul pas cu pas

1. **Inițializare (seed)** — la prima citire validă a accelerometrului, se calculează roll/pitch din gravitate:
   ```
   roll  = atan2(ay, az)
   pitch = atan2(-ax, √(ay² + az²))
   ```
   Se construiește quaternionul inițial din acești unghiuri Euler.

2. **Predicție (integare giroscop)** — derivata quaternionului:
   ```
   dq₀ = 0.5 × (−q₁ωx − q₂ωy − q₃ωz)
   dq₁ = 0.5 × ( q₀ωx + q₂ωz − q₃ωy)
   dq₂ = 0.5 × ( q₀ωy − q₁ωz + q₃ωx)
   dq₃ = 0.5 × ( q₀ωz + q₁ωy − q₂ωx)
   ```
   Integrare Euler: `q += dq × dt`

3. **Corecție accelerometru** — feedback proportional (tip Mahony):
   ```
   error = accel_normalizat × gravity_estimat  (cross product)
   ω_corectat += QUAT_ACC_GAIN × error
   ```
   `QUAT_ACC_GAIN = 1.8`

4. **Corecție magnetometru (dacă prezent)** — tilt-compensated heading:
   ```
   mx_comp = mx×cos(pitch) + mz×sin(pitch)
   my_comp = mx×sin(roll)×sin(pitch) + my×cos(roll) − mz×sin(roll)×cos(pitch)
   heading = atan2(my_comp, mx_comp)
   ```
   Corecție yaw cu blend dinamic:
   - La giroscop lent (<220 dps): blend mare (`YAW_MAG_BLEND_MAX = 0.14`)
   - La giroscop rapid (>220 dps): blend mic (`YAW_MAG_BLEND_MIN = 0.03`)
   - Corecție adițională: `QUAT_MAG_GAIN × 0.01 = 0.014`

5. **Normalizare quaternion** după fiecare pas.

6. **Extragere Euler din quaternion:**
   ```
   roll  = atan2(2(q₀q₁ + q₂q₃), 1 − 2(q₁² + q₂²))
   pitch = asin(2(q₀q₂ − q₃q₁))
   yaw   = atan2(2(q₀q₃ + q₁q₂), 1 − 2(q₂² + q₃²))
   ```

### Exemplu de date JSON (WebSocket, ~60 Hz)

```json
{
  "q0": 0.9998, "q1": 0.0012, "q2": -0.0031, "q3": 0.0184,
  "roll": 0.14, "pitch": -0.36, "yaw": 2.11,
  "stationary": true, "mode": "fusion",
  "frame": "right-handed", "quat_order": "wxyz"
}
```

### Nefolosit
- **Constantele Kalman** (`KALMAN_Q_ANGLE`, `KALMAN_Q_BIAS`, `KALMAN_R_MEASURE`) — definite în `config.h` dar **nefolosite** de codul nou `fusion.cpp`. Erau utilizate de filtrul Kalman din `main.cpp` (fișierul vechi monolitic).
- **`FusionState::dmpActive`** — câmpul există dar este mereu `false`

### Îmbunătățiri posibile
- Implementare filtru Kalman complet (Extended Kalman Filter) în loc de complementary filter
- Gradual correction (SLERP) în loc de corecție Euler bruscă pe yaw
- Ajustare adaptivă `QUAT_ACC_GAIN` în funcție de magnitudinea accelerației (reduce drift la vibrații)

---

## 6. Estimare mișcare (poziție/viteză)

**Fișiere:** `src/motion/fusion.cpp` (funcția `updateMotionEstimate`)

### Cum funcționează

Integrare dublă a accelerației liniare (fără gravitație) pentru obținerea vitezei și poziției.

### Formule

1. **Gravitație estimată din quaternion:**
   ```
   gravX = 2(q₁q₃ − q₀q₂)
   gravY = 2(q₀q₁ + q₂q₃)
   gravZ = q₀² − q₁² − q₂² + q₃²
   ```

2. **Accelerație liniară** = accelerație måsurată − gravitație

3. **Rotire body→world** folosind matricea de rotație derivată din quaternion

4. **Integrare viteză** (cu damping 0.5%/step):
   ```
   vel = (vel + linAcc × dt) × 0.995
   ```

5. **Integrare poziție:**
   ```
   pos += vel × dt
   ```

### ZUPT (Zero Velocity Update)

Când `motionStillCount >= 10` (accelerație liniară < 0.08g):
- **Viteza se resetează la 0**
- **Poziția convergește lent** la o ancoră fixă:
  ```
  pos = pos × (1 − 0.002) + anchor × 0.002
  ```

### Exemplu de date JSON

```json
{
  "lin_ax": 0.003, "lin_ay": -0.012, "lin_az": 0.089,
  "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,
  "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
  "dt_ms": 9.87, "stationary": true, "zupt": true
}
```

### Îmbunătățiri posibile
- Poziția din integrare dublă a IMU se degradează rapid (drift) — limitare practică la ~10 secunde
- Ar putea folosi GPS sau UWB pentru corecție periodică
- Filtru trapezoidal (în loc de Euler) pentru integrare numerică mai precisă

---

## 7. Baterie — Coulomb counting + OCV hibrid

**Fișiere:** `src/system/battery.cpp`, `src/system/battery.h`

### Cum funcționează

Estimare SOC (State of Charge) prin două metode combinate:

### Librărie

**Preferences** (ESP32 NVS) — pentru persistarea stării bateriei.

### Metoda 1: Coulomb counting

```
totalMAh += curent_mA × (dt_ms / 3 600 000)
SOC_count = 100 − (totalMAh / BATTERY_CAPACITY) × 100
```
`BATTERY_CAPACITY = 3200 mAh`

### Metoda 2: OCV (Open Circuit Voltage) lookup

Tabel de interpolare liniară cu 11 puncte:

| Tensiune (V) | SOC (%) |
|--------------|---------|
| 4.20 | 100 |
| 4.10 | 90 |
| 4.00 | 75 |
| 3.90 | 60 |
| 3.80 | 45 |
| 3.70 | 30 |
| 3.60 | 18 |
| 3.50 | 10 |
| 3.40 | 5 |
| 3.30 | 2 |
| 3.20 | 0 |

### Fuziune SOC

Greutatea tabelei OCV variază cu starea bateriei:
- Normal: `ocvWeight = 0.05`
- Curent scăzut (<25 mA): `ocvWeight = 0.15`
- Repaus lung (>30 s la curent scăzut): `ocvWeight = 0.35`
- Discrepanță mare (>20% între metode): `ocvWeight = 0.50`

```
SOC = (1 − ocvWeight) × SOC_counting + ocvWeight × SOC_ocv
```

### Bootstrap

Dacă nu există date persistate, SOC inițial se estimează prim OCV la boot.

### Viață estimată

```
remainingMAh = BATTERY_CAPACITY × (SOC / 100)
lifeMinutes = (remainingMAh / |curent_mA|) × 60
```

### Detecție prezență baterie

Debounce de 5 secunde:
- Baterie deconectată: V ≤ 2.4V pentru 5s
- Baterie reconectată: V ≥ 2.8V pentru 5s

### Persistare

Se salvează `totalMAh` la fiecare 60 de secunde în NVS (`Preferences`).

### Exemplu de date JSON

```json
{ "batt": 78.5, "mah": 688.0, "batt_min": 342 }
```

### Îmbunătățiri posibile
- Compensare temperatură pe tabelul OCV (capacitatea Li-Ion scade ~20% la 0°C)
- Curba OCV diferită pentru descărcare vs încărcare (histerezis)
- Alertă la SOC scăzut (e.g., LED blink, mesaj MQTT)

---

## 8. RTC — DS1307

**Fișiere:** `src/system/rtc_time.cpp`, `src/system/rtc_time.h`

### Cum funcționează

1. Verifică prezența RTC la adresele `0x68` (DS1307 standard) sau `0x60` (alternativ)
2. Dacă RTC nu rulează, setează data/ora compilării
3. La fiecare secundă, actualizează timestamp-ul cu prioritate:
   - **NTP** (dacă WiFi conectat) → `getLocalTime()`
   - **RTC** (dacă prezent) → citire directă registre BCD
   - **Uptime** (fallback) → `1970-01-01 HH:MM:SS`

### Librărie

**RTClib** (`RTClib @ 2.1.4`) — doar pentru DS1307 la adresa standard. Citirea registrelor BCD se face și manual pentru flexibilitate.

### Formule BCD

```
bcdToBin(0x23) → 23   // (2×10 + 3)
binToBcd(23)   → 0x23 // ((23/10)<<4 | 23%10)
```

### Sincronizare NTP → RTC

După conectare WiFi, se apelează `rtc_sync_from_system()` care scrie ora NTP în DS1307.

### Exemplu de timestamp

```
"2026-04-02 14:23:07"
```

### Îmbunătățiri posibile
- Suport pentru DS3231 (mai precis, ±2 ppm vs ±100 ppm la DS1307)
- Citire temperatură din DS3231 (senzor integrat)
- Timezone configurabil (actualmente depinde de NTP server)

---

## 9. Monitor CPU

**Fișiere:** `src/system/cpu_monitor.cpp`, `src/system/cpu_monitor.h`

### Cum funcționează

Două metode complementare de măsurare:

### Metoda 1: FreeRTOS Runtime Stats (dacă disponibilă)

```
cpuLoad = 100 − (100 × idleTime / totalTime)
```
Folosește `uxTaskGetSystemState()` și caută task-ul "IDLE".

### Metoda 2: Idle Hook (fallback)

Un hook înregistrat pe taskul IDLE numără iterațiile per secundă:
```
idleRate = idleDelta / elapsed_ms
cpuLoad = 100 − (100 × idleRate / idleRateRef)
```
`idleRateRef` se adaptează lent: dacă `idleRate > ref` → ref = idleRate, altfel `ref *= 0.999 + idleRate * 0.001`.

### Filtru de afișare

Filtru EMA asimetric cu rate-limiting:
- Alpha pentru creștere: 0.35
- Alpha pentru scădere: 0.20
- Pas maxim: ±3% per eșantion
- Sub 0.3% → afișat ca 0%

### CPU Stress Task

Un task FreeRTOS dedicat (`cpu_stress`) care execută calcule intense (sqrt + log) când este activat prin comanda MQTT `cpu_stress`. Creat cu `xTaskCreate()`, prioritate 1, stack 3072 bytes.

### Îmbunătățiri posibile
- Afișare per-task CPU usage (informația e disponibilă din `uxTaskGetSystemState`)
- Alertă MQTT dacă CPU > 90% pentru o perioadă lungă

---

## 10. Monitor GPIO

**Fișiere:** `src/data/gpio_monitor.cpp`, `src/data/gpio_monitor.h`

### Cum funcționează

Monitorizează 13 pini GPIO la fiecare secundă:
```
GPIO_PUBLISH_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21}
```
(Evită GPIO18/19 = USB D-/D+ pe ESP32-C3)

Detectează schimbări de stare și le loghează într-un buffer circular (5 linii × 21 caractere) afișat pe OLED.

### Exemplu log OLED

```
#042 G03 BTN+ ON
#043 G04 BTN- OFF
TH RAW:2187 T:24.3
GYR X:  -1 Y:   0
GYR Z:   0 INA: 3120
```

### Exemplu date JSON (MQTT raw)

```json
{
  "gpio": {
    "gpio0": 1, "gpio1": 1, "gpio2": 0, "gpio3": 1,
    "gpio4": 1, "gpio5": 0, "gpio6": 1, "gpio7": 0,
    "gpio8": 1, "gpio9": 1, "gpio10": 1, "gpio20": 0, "gpio21": 1
  }
}
```

### Îmbunătățiri posibile
- Configurare pini monitorizate prin MQTT (nu hardcodat)
- Timestamps per eveniment GPIO (nu doar la fiecare secundă)
- Mod interrupt pentru detectare rapidă de evenimente

---

## 11. Display OLED SSD1306

**Fișiere:** `src/ui/display.cpp`, `src/ui/display.h`

### Cum funcționează

Display 128×64 pixeli I2C, cu 10 pagini navigabile prin butoane fizice (GPIO3 = next, GPIO4 = prev).

### Librării

- **Adafruit SSD1306** (`@ 2.5.16`)
- **Adafruit GFX Library** (`@ 1.12.5`)
- **QRCode** (`@ 0.0.1`) — pentru generarea codului QR WiFi pe pagina hotspot

### Paginile

| # | Nume | Conținut |
|---|------|---------|
| 1 | MONITOR | Dashboard: oră, temperatură, giroscop, putere, RSSI, CPU, bară baterie |
| 2 | TEMPERATURA | Temperatură mare (font 3x) + bară % |
| 3 | GIROSCOP | Axe X/Y/Z (dps) + abs + bară |
| 4 | RETEA + MQTT | SSID, IP, RSSI, dată, status MQTT + bare semnal |
| 5 | INC. CPU | Încărcare CPU mare (font 3x) + bară |
| 6 | TENSIUNE | Tensiune (font 3x) + putere mW + bară |
| 7 | ORIENTARE | Roll/Pitch/Yaw + heading + quaternion |
| 8 | CURENT | mA (font 3x) + mAh, baterie %, viață |
| 9 | I2C SCAN | Scanare bus I2C + lista dispozitivelor găsite |
| 10 | GPIO LOG | Buffer circular cu ultimele 5 evenimente |

### Debounce

Butoane cu ISR pe FALLING + debounce 200 ms + ignore period de 200 ms după orice apăsare.

---

## 12. Constante definite dar nefolosite în codul nou

Următoarele constante sunt definite în `config.h` dar **nu sunt referite** în niciun fișier din `src/` (modulele noi). Ele erau folosite doar de `main.cpp` (fișierul monolitic vechi):

| Constantă | Valoare | Scop original |
|-----------|---------|---------------|
| `THERM_NTC_TO_GND` | `true` | Topologia termistorului |
| `KALMAN_Q_ANGLE` | 0.001 | Filtru Kalman — zgomot proces unghi |
| `KALMAN_Q_BIAS` | 0.003 | Filtru Kalman — zgomot proces bias |
| `KALMAN_R_MEASURE` | 0.03 | Filtru Kalman — zgomot măsurare |

> **Notă:** `QUAT_MAG_GAIN`, `YAW_MAG_BLEND_MIN`, `YAW_MAG_BLEND_MAX` **sunt** folosite de `fusion.cpp`.

---

## 13. Rezumat îmbunătățiri posibile

| Prioritate | Modul | Îmbunătățire |
|-----------|-------|--------------|
| **Înaltă** | IMU/LSM6DS3 | Citire accelerometru pe LSM (actualmente 0) |
| **Înaltă** | Magnetometru | Calibrare hard-iron / soft-iron |
| **Medie** | Termistor | Implementare `THERM_NTC_TO_GND` |
| **Medie** | Baterie | Compensare temperatură pe curba OCV |
| **Medie** | Fusion | EKF complet sau Madgwick/Mahony cu gain adaptiv |
| **Medie** | IMU | Activare DMP pe MPU pentru offload CPU |
| **Scăzută** | Config | Ștergere constantele Kalman nefolosite |
| **Scăzută** | CPU | Per-task CPU breakdown |
| **Scăzută** | GPIO | Mod interrupt pentru detectare rapidă |
| **Scăzută** | RTC | Suport DS3231 |
