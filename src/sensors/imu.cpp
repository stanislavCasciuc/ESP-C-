#include "imu.h"
#include "config.h"
#include "../motion/fusion.h"

#include <Arduino.h>
#include <Wire.h>

// I2C helpers
static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t rd = Wire.requestFrom(addr, (uint8_t)len);
    if (rd < len) return false;
    for (size_t i = 0; i < len; i++) data[i] = Wire.read();
    return true;
}

bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// State
static ImuData sData = {};
static uint8_t sImuAddr = 0;
static bool sImuIsMpu = false;
static bool sHasMag = false;
static bool sMpuDmpActive = false;

static float sGyroBiasX = 0, sGyroBiasY = 0, sGyroBiasZ = 0;
static float sAccelBiasX = 0, sAccelBiasY = 0, sAccelBiasZ = 0;
static bool  sAccelBiasSeeded = false;
static bool  sGyroFilterSeeded = false;
static uint8_t sGyroStillCount = 0;

static float sMagAdjX = 1.0f, sMagAdjY = 1.0f, sMagAdjZ = 1.0f;
static uint32_t sLastMagReadMs = 0;

static volatile uint16_t sGyroDataReadyCount = 0;

void IRAM_ATTR imu_isr_data_ready() {
    if (sGyroDataReadyCount < 1000) sGyroDataReadyCount++;
}

uint16_t imu_consume_data_ready_count() {
    noInterrupts();
    uint16_t c = sGyroDataReadyCount;
    sGyroDataReadyCount = 0;
    interrupts();
    return c;
}

const ImuData& imu_get_data() { return sData; }
uint8_t imu_get_addr() { return sImuAddr; }
bool imu_is_mpu() { return sImuIsMpu; }
bool imu_has_mag() { return sHasMag; }
bool imu_is_dmp_active() { return sMpuDmpActive; }

static bool calibrateMpuGyroBias(uint8_t mpuAddr) {
    int32_t sx = 0, sy = 0, sz = 0;
    uint8_t buf[6] = {0};

    for (uint8_t i = 0; i < MPU_BIAS_SAMPLES; i++) {
        if (!i2cReadRegs(mpuAddr, MPU_GYRO_XOUT_H, buf, sizeof(buf))) return false;
        sx += (int16_t)((buf[0] << 8) | buf[1]);
        sy += (int16_t)((buf[2] << 8) | buf[3]);
        sz += (int16_t)((buf[4] << 8) | buf[5]);
        delay(3);
    }
    sGyroBiasX = (float)sx / MPU_BIAS_SAMPLES;
    sGyroBiasY = (float)sy / MPU_BIAS_SAMPLES;
    sGyroBiasZ = (float)sz / MPU_BIAS_SAMPLES;
    return true;
}

static bool readMagnetometer() {
    if (!sHasMag) return false;

    uint8_t st1 = 0;
    if (!i2cReadRegs(AK8963_ADDR, AK8963_ST1, &st1, 1)) return false;
    if ((st1 & 0x01) == 0) return false;

    uint8_t data[7] = {0};
    if (!i2cReadRegs(AK8963_ADDR, AK8963_XOUT_L, data, 7)) return false;
    if (data[6] & 0x08) return false; // overflow

    sData.magRawX = (int16_t)((data[1] << 8) | data[0]);
    sData.magRawY = (int16_t)((data[3] << 8) | data[2]);
    sData.magRawZ = (int16_t)((data[5] << 8) | data[4]);

    sData.mx = sData.magRawX * 0.15f * sMagAdjX;
    sData.my = sData.magRawY * 0.15f * sMagAdjY;
    sData.mz = sData.magRawZ * 0.15f * sMagAdjZ;
    return true;
}

bool imu_init() {
    uint8_t who = 0;
    uint8_t mpuAddr = 0;

    if (i2cProbe(IMU_ADDR_MPU1) && i2cReadRegs(IMU_ADDR_MPU1, MPU_WHO_AM_I_REG, &who, 1)) {
        mpuAddr = IMU_ADDR_MPU1;
    } else if (i2cProbe(IMU_ADDR_MPU0) && i2cReadRegs(IMU_ADDR_MPU0, MPU_WHO_AM_I_REG, &who, 1)) {
        mpuAddr = IMU_ADDR_MPU0;
    }

    if (mpuAddr != 0 && (who == MPU_WHO_AM_I_6500 || who == MPU_WHO_AM_I_9250)) {
        if (!i2cWriteReg(mpuAddr, MPU_PWR_MGMT_1, 0x01)) return false;
        if (!i2cWriteReg(mpuAddr, MPU_PWR_MGMT_2, 0x00)) return false;
        delay(20);
        if (!i2cWriteReg(mpuAddr, MPU_CONFIG_REG, 0x04)) return false;
        if (!i2cWriteReg(mpuAddr, MPU_SMPLRT_DIV, 0x00)) return false;
        if (!i2cWriteReg(mpuAddr, MPU_GYRO_CONFIG, 0x08)) return false;
        if (!i2cWriteReg(mpuAddr, MPU_ACCEL_CONFIG, 0x00)) return false;
        if (!i2cWriteReg(mpuAddr, MPU_INT_PIN_CFG, 0x02)) return false;
        if (!i2cWriteReg(mpuAddr, MPU_INT_ENABLE, 0x01)) return false;
        (void)calibrateMpuGyroBias(mpuAddr);

        // Magnetometer (AK8963 on MPU9250)
        sHasMag = false;
        uint8_t magWho = 0;
        if (i2cProbe(AK8963_ADDR) && i2cReadRegs(AK8963_ADDR, AK8963_WHO_AM_I, &magWho, 1) && magWho == AK8963_WHO_AM_I_VAL) {
            if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x00)) {
                delay(10);
                if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x0F)) {
                    delay(10);
                    uint8_t asa[3] = {0};
                    if (i2cReadRegs(AK8963_ADDR, AK8963_ASAX, asa, 3)) {
                        sMagAdjX = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;
                        sMagAdjY = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;
                        sMagAdjZ = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;
                    }
                }
            }
            if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x00)) {
                delay(10);
                if (i2cWriteReg(AK8963_ADDR, AK8963_CNTL1, 0x16)) {
                    delay(10);
                    sHasMag = true;
                }
            }
        }

        sImuAddr = mpuAddr;
        sImuIsMpu = true;
        sMpuDmpActive = false;
        sGyroFilterSeeded = false;
        sData.mag_present = sHasMag;
        fusion_reset();
        return true;
    }

    // Fallback LSM6DS3
    if (!i2cProbe(IMU_ADDR_LSM6)) return false;
    if (!i2cReadRegs(IMU_ADDR_LSM6, LSM_WHO_AM_I_REG, &who, 1)) return false;
    if (who != LSM_WHO_AM_I_VAL) return false;
    if (!i2cWriteReg(IMU_ADDR_LSM6, LSM_CTRL2_G, 0x4C)) return false;
    delay(10);
    sImuAddr = IMU_ADDR_LSM6;
    sImuIsMpu = false;
    sHasMag = false;
    sData.mag_present = false;
    fusion_reset();
    return true;
}

bool imu_read() {
    if (sImuAddr == 0) return false;

    if (sImuIsMpu) {
        uint8_t buf[14] = {0};
        if (!i2cReadRegs(sImuAddr, MPU_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

        sData.accelRawX = (int16_t)((buf[0] << 8) | buf[1]);
        sData.accelRawY = (int16_t)((buf[2] << 8) | buf[3]);
        sData.accelRawZ = (int16_t)((buf[4] << 8) | buf[5]);
        sData.gyroRawX  = (int16_t)((buf[8] << 8) | buf[9]);
        sData.gyroRawY  = (int16_t)((buf[10] << 8) | buf[11]);
        sData.gyroRawZ  = (int16_t)((buf[12] << 8) | buf[13]);

        float gx = ((float)sData.gyroRawX - sGyroBiasX) * MPU_GYRO_SCALE_DPS;
        float gy = ((float)sData.gyroRawY - sGyroBiasY) * MPU_GYRO_SCALE_DPS;
        float gz = ((float)sData.gyroRawZ - sGyroBiasZ) * MPU_GYRO_SCALE_DPS;

        float absRate = sqrtf(gx * gx + gy * gy + gz * gz);
        if (absRate < GYRO_STILL_DPS) {
            if (sGyroStillCount < 255) sGyroStillCount++;
        } else {
            sGyroStillCount = 0;
        }

        float axMeas = sData.accelRawX * MPU_ACCEL_SCALE_G;
        float ayMeas = sData.accelRawY * MPU_ACCEL_SCALE_G;
        float azMeas = sData.accelRawZ * MPU_ACCEL_SCALE_G;

        sData.ax = axMeas - sAccelBiasX;
        sData.ay = ayMeas - sAccelBiasY;
        sData.az = azMeas - sAccelBiasZ;

        if (!sGyroFilterSeeded) {
            sData.gx = gx; sData.gy = gy; sData.gz = gz;
            sGyroFilterSeeded = true;
        } else {
            sData.gx += GYRO_FILTER_ALPHA * (gx - sData.gx);
            sData.gy += GYRO_FILTER_ALPHA * (gy - sData.gy);
            sData.gz += GYRO_FILTER_ALPHA * (gz - sData.gz);
        }

        if (fabsf(sData.gx) < GYRO_DEADBAND_DPS) sData.gx = 0.0f;
        if (fabsf(sData.gy) < GYRO_DEADBAND_DPS) sData.gy = 0.0f;
        if (fabsf(sData.gz) < GYRO_DEADBAND_DPS) sData.gz = 0.0f;

        if (sHasMag && (millis() - sLastMagReadMs >= 10)) {
            (void)readMagnetometer();
            sLastMagReadMs = millis();
        }

        // Run fusion
        fusion_update(gx, gy, gz, sData.ax, sData.ay, sData.az,
                      sData.mx, sData.my, sData.mz, sHasMag);

        // Update biases from fusion state
        const FusionState& fs = fusion_get_state();
        bool stationary = fs.motionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES;
        if (stationary) {
            uint32_t nowMs = millis();
            static uint32_t sLastBiasUpdateMs = 0;
            if (sLastBiasUpdateMs == 0 || nowMs - sLastBiasUpdateMs >= BIAS_RECAL_INTERVAL_MS) {
                sLastBiasUpdateMs = nowMs;
                float gyroAlpha = (fs.motionStillCount >= (ZUPT_STILL_REQUIRED_SAMPLES * 2)) ? GYRO_BIAS_RECAL_ALPHA : GYRO_BIAS_STILL_ALPHA;
                float accelAlpha = (fs.motionStillCount >= (ZUPT_STILL_REQUIRED_SAMPLES * 2)) ? ACCEL_BIAS_RECAL_ALPHA : ACCEL_BIAS_STILL_ALPHA;

                if (!sAccelBiasSeeded) {
                    sAccelBiasX = axMeas - fs.gravityX;
                    sAccelBiasY = ayMeas - fs.gravityY;
                    sAccelBiasZ = azMeas - fs.gravityZ;
                    sAccelBiasSeeded = true;
                } else {
                    sGyroBiasX += gyroAlpha * (gx - sGyroBiasX);
                    sGyroBiasY += gyroAlpha * (gy - sGyroBiasY);
                    sGyroBiasZ += gyroAlpha * (gz - sGyroBiasZ);
                    sAccelBiasX += accelAlpha * ((axMeas - fs.gravityX) - sAccelBiasX);
                    sAccelBiasY += accelAlpha * ((ayMeas - fs.gravityY) - sAccelBiasY);
                    sAccelBiasZ += accelAlpha * ((azMeas - fs.gravityZ) - sAccelBiasZ);
                }
            }
        }
    } else {
        // LSM6DS3
        uint8_t buf[6] = {0};
        if (!i2cReadRegs(sImuAddr, (uint8_t)(LSM_OUTX_L_G | 0x80), buf, sizeof(buf))) return false;

        sData.gyroRawX = (int16_t)((buf[1] << 8) | buf[0]);
        sData.gyroRawY = (int16_t)((buf[3] << 8) | buf[2]);
        sData.gyroRawZ = (int16_t)((buf[5] << 8) | buf[4]);
        sData.accelRawX = sData.accelRawY = sData.accelRawZ = 0;

        sData.gx = sData.gyroRawX * LSM_GYRO_SCALE_DPS;
        sData.gy = sData.gyroRawY * LSM_GYRO_SCALE_DPS;
        sData.gz = sData.gyroRawZ * LSM_GYRO_SCALE_DPS;
        sData.ax = sData.ay = sData.az = 0.0f;

        fusion_update(sData.gx, sData.gy, sData.gz, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, false);
    }

    sData.gyroAbs = sqrtf(sData.gx * sData.gx + sData.gy * sData.gy + sData.gz * sData.gz);
    sData.mag_present = sHasMag;
    return true;
}
