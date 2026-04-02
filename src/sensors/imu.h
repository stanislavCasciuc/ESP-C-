#pragma once

#include <Arduino.h>
#include <cstdint>

struct ImuData {
    int16_t gyroRawX, gyroRawY, gyroRawZ;
    int16_t accelRawX, accelRawY, accelRawZ;
    int16_t magRawX, magRawY, magRawZ;
    float gx, gy, gz;       // gyroscope (deg/s) filtered
    float ax, ay, az;       // accelerometer (g) bias-corrected
    float mx, my, mz;       // magnetometer (uT)
    float gyroAbs;
    bool  mag_present;
};

// Init the IMU (tries MPU6500/9250 then LSM6DS3). Returns true if found.
bool imu_init();

// Read gyro/accel/mag. Runs fusion + motion internally. Returns true on success.
bool imu_read();

// Get the last IMU data
const ImuData& imu_get_data();

// Accessors for setup info
uint8_t imu_get_addr();
bool    imu_is_mpu();
bool    imu_has_mag();
bool    imu_is_dmp_active();

// ISR for data-ready interrupt
void IRAM_ATTR imu_isr_data_ready();
uint16_t imu_consume_data_ready_count();
