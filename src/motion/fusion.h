#pragma once

#include <cstdint>

struct FusionState {
    float q0, q1, q2, q3;      // quaternion wxyz
    float roll, pitch, yaw;     // Euler (degrees)
    float heading;
    float linAccX, linAccY, linAccZ;  // gravity-removed, world frame (m/s^2)
    float velX, velY, velZ;
    float posX, posY, posZ;
    float gravityX, gravityY, gravityZ;
    float gyroBiasX, gyroBiasY, gyroBiasZ;
    float accelBiasX, accelBiasY, accelBiasZ;
    float motionDtMs;
    uint8_t motionStillCount;
    bool stationary;
    bool dmpActive;
};

void fusion_reset();
void fusion_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   bool hasMag);
const FusionState& fusion_get_state();
