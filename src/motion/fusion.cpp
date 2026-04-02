#include "fusion.h"
#include "config.h"

#include <Arduino.h>
#include <cmath>

static FusionState sState = {};
static bool sFusionSeeded = false;
static uint32_t sLastFusionUs = 0;
static uint32_t sLastMotionUs = 0;
static bool sMotionSeeded = false;
static bool sMotionAnchorValid = false;
static float sMotionAnchorX = 0, sMotionAnchorY = 0, sMotionAnchorZ = 0;

static float wrapAngle180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static float normalize360(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
}

static void updateEulerFromQuaternion() {
    const float q0 = sState.q0, q1 = sState.q1, q2 = sState.q2, q3 = sState.q3;
    sState.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    const float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sState.pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * 57.29578f;
    sState.yaw   = normalize360(atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f);
}

static void updateMotionEstimate(float dt) {
    if (dt <= 0.0f) return;
    sState.motionDtMs = dt * 1000.0f;

    const float q0 = sState.q0, q1 = sState.q1, q2 = sState.q2, q3 = sState.q3;
    float gravX = 2.0f * (q1 * q3 - q0 * q2);
    float gravY = 2.0f * (q0 * q1 + q2 * q3);
    float gravZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    sState.gravityX = gravX;
    sState.gravityY = gravY;
    sState.gravityZ = gravZ;

    // Linear acceleration: body frame -> world frame
    float linBodyX = sState.accelBiasX - gravX; // note: ax already bias corrected coming in
    float linBodyY = sState.accelBiasY - gravY;
    float linBodyZ = sState.accelBiasZ - gravZ;

    // Use the actual accelerometer values stored temporarily
    // We need the raw accel values here. They're passed through fusion_update.
    // Store them in state for this purpose.

    float r00 = 1.0f - 2.0f * (q2*q2 + q3*q3);
    float r01 = 2.0f * (q1*q2 - q0*q3);
    float r02 = 2.0f * (q1*q3 + q0*q2);
    float r10 = 2.0f * (q1*q2 + q0*q3);
    float r11 = 1.0f - 2.0f * (q1*q1 + q3*q3);
    float r12 = 2.0f * (q2*q3 - q0*q1);
    float r20 = 2.0f * (q1*q3 - q0*q2);
    float r21 = 2.0f * (q2*q3 + q0*q1);
    float r22 = 1.0f - 2.0f * (q1*q1 + q2*q2);

    float worldLinGx = r00 * linBodyX + r01 * linBodyY + r02 * linBodyZ;
    float worldLinGy = r10 * linBodyX + r11 * linBodyY + r12 * linBodyZ;
    float worldLinGz = r20 * linBodyX + r21 * linBodyY + r22 * linBodyZ;

    constexpr float G_TO_MPS2 = 9.80665f;
    sState.linAccX = worldLinGx * G_TO_MPS2;
    sState.linAccY = worldLinGy * G_TO_MPS2;
    sState.linAccZ = worldLinGz * G_TO_MPS2;

    float accMagG = sqrtf(worldLinGx*worldLinGx + worldLinGy*worldLinGy + worldLinGz*worldLinGz);
    // We need gyroAbs for stationarity check. Approximate from recent rates.
    bool nearlyStill = accMagG < STATIONARY_WORLD_ACC_G;

    if (!sMotionSeeded) {
        sMotionSeeded = true;
        sState.velX = sState.velY = sState.velZ = 0;
        sState.posX = sState.posY = sState.posZ = 0;
    }

    if (nearlyStill) {
        if (sState.motionStillCount < 255) sState.motionStillCount++;
    } else {
        sState.motionStillCount = 0;
    }

    bool zuptActive = (sState.motionStillCount >= ZUPT_STILL_REQUIRED_SAMPLES);
    sState.stationary = zuptActive;

    if (zuptActive) {
        if (!sMotionAnchorValid) {
            sMotionAnchorX = sState.posX;
            sMotionAnchorY = sState.posY;
            sMotionAnchorZ = sState.posZ;
            sMotionAnchorValid = true;
        }
        sState.velX = sState.velY = sState.velZ = 0;
        sState.posX = sState.posX * (1.0f - ZUPT_POS_HOLD_ALPHA) + sMotionAnchorX * ZUPT_POS_HOLD_ALPHA;
        sState.posY = sState.posY * (1.0f - ZUPT_POS_HOLD_ALPHA) + sMotionAnchorY * ZUPT_POS_HOLD_ALPHA;
        sState.posZ = sState.posZ * (1.0f - ZUPT_POS_HOLD_ALPHA) + sMotionAnchorZ * ZUPT_POS_HOLD_ALPHA;
    } else {
        sMotionAnchorValid = false;
        sState.velX = (sState.velX + sState.linAccX * dt) * 0.995f;
        sState.velY = (sState.velY + sState.linAccY * dt) * 0.995f;
        sState.velZ = (sState.velZ + sState.linAccZ * dt) * 0.995f;
        sState.posX += sState.velX * dt;
        sState.posY += sState.velY * dt;
        sState.posZ += sState.velZ * dt;
    }

    if (!isfinite(sState.posX)) sState.posX = 0;
    if (!isfinite(sState.posY)) sState.posY = 0;
    if (!isfinite(sState.posZ)) sState.posZ = 0;
}

void fusion_reset() {
    sFusionSeeded = false;
    sMotionSeeded = false;
    sMotionAnchorValid = false;
    sLastFusionUs = 0;
    sLastMotionUs = 0;
    sState = {};
    sState.q0 = 1.0f;
    sState.gravityZ = 1.0f;
}

void fusion_update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   bool hasMag) {
    const float DEG_TO_RAD_F = 0.0174532925f;
    float wx = gx * DEG_TO_RAD_F;
    float wy = gy * DEG_TO_RAD_F;
    float wz = gz * DEG_TO_RAD_F;

    float accNorm = sqrtf(ax*ax + ay*ay + az*az);
    bool accelValid = accNorm > 0.3f;

    float q0 = sState.q0, q1 = sState.q1, q2 = sState.q2, q3 = sState.q3;

    // Compute fusion dt
    uint32_t nowUs = micros();
    float dt = 0.005f;
    if (sLastFusionUs != 0) {
        dt = (nowUs - sLastFusionUs) * 1e-6f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;
    }
    sLastFusionUs = nowUs;

    if (!sFusionSeeded) {
        if (accelValid) {
            float invAcc = 1.0f / accNorm;
            float axn = ax * invAcc, ayn = ay * invAcc, azn = az * invAcc;
            sState.roll  = atan2f(ayn, azn) * 57.29578f;
            sState.pitch = atan2f(-axn, sqrtf(ayn*ayn + azn*azn)) * 57.29578f;

            float hr = sState.roll * 0.00872664626f;
            float hp = sState.pitch * 0.00872664626f;
            float cr = cosf(hr), sr = sinf(hr);
            float cp = cosf(hp), sp = sinf(hp);
            q0 = cr*cp + sr*sp*0; // cy=1, sy=0
            q1 = sr*cp;
            q2 = cr*sp;
            q3 = 0; // -sr*sp
        }
        sFusionSeeded = true;
    }

    if (accelValid) {
        float invAcc = 1.0f / accNorm;
        float axn = ax*invAcc, ayn = ay*invAcc, azn = az*invAcc;
        float gvX = 2.0f * (q1*q3 - q0*q2);
        float gvY = 2.0f * (q0*q1 + q2*q3);
        float gvZ = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        float ex = ayn*gvZ - azn*gvY;
        float ey = azn*gvX - axn*gvZ;
        float ez = axn*gvY - ayn*gvX;
        wx += QUAT_ACC_GAIN * ex;
        wy += QUAT_ACC_GAIN * ey;
        wz += QUAT_ACC_GAIN * ez;
    }

    float dq0 = 0.5f * (-q1*wx - q2*wy - q3*wz);
    float dq1 = 0.5f * ( q0*wx + q2*wz - q3*wy);
    float dq2 = 0.5f * ( q0*wy - q1*wz + q3*wx);
    float dq3 = 0.5f * ( q0*wz + q1*wy - q2*wx);

    q0 += dq0 * dt; q1 += dq1 * dt; q2 += dq2 * dt; q3 += dq3 * dt;

    float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (qNorm > 1e-6f) { q0/=qNorm; q1/=qNorm; q2/=qNorm; q3/=qNorm; }
    else { q0=1; q1=q2=q3=0; }

    sState.q0=q0; sState.q1=q1; sState.q2=q2; sState.q3=q3;
    updateEulerFromQuaternion();

    // Mag-based yaw correction
    if (hasMag) {
        float rollRad = sState.roll * DEG_TO_RAD_F;
        float pitchRad = sState.pitch * DEG_TO_RAD_F;
        float mxComp = mx*cosf(pitchRad) + mz*sinf(pitchRad);
        float myComp = mx*sinf(rollRad)*sinf(pitchRad) + my*cosf(rollRad) - mz*sinf(rollRad)*cosf(pitchRad);
        sState.heading = normalize360(atan2f(myComp, mxComp) * 57.29578f);

        float yawErr = wrapAngle180(sState.heading - sState.yaw);
        float absRate = sqrtf(gx*gx + gy*gy + gz*gz);
        float dynamicBlend = YAW_MAG_BLEND_MAX;
        if (absRate > 220.0f) dynamicBlend = YAW_MAG_BLEND_MIN;
        else if (absRate > 0.0f) {
            float t = absRate / 220.0f;
            dynamicBlend = YAW_MAG_BLEND_MAX - (YAW_MAG_BLEND_MAX - YAW_MAG_BLEND_MIN) * t;
        }
        float yawCorr = (dynamicBlend + QUAT_MAG_GAIN * 0.01f) * yawErr;
        float halfYaw = 0.5f * yawCorr * DEG_TO_RAD_F;
        float cq = cosf(halfYaw), sq = sinf(halfYaw);
        float nq0 = q0*cq - q3*sq;
        float nq1 = q1*cq - q2*sq;
        float nq2 = q2*cq + q1*sq;
        float nq3 = q3*cq + q0*sq;
        float nNorm = sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
        if (nNorm > 1e-6f) {
            sState.q0=nq0/nNorm; sState.q1=nq1/nNorm;
            sState.q2=nq2/nNorm; sState.q3=nq3/nNorm;
            updateEulerFromQuaternion();
        }
    } else {
        sState.heading = sState.yaw;
    }

    // Store accel values for motion estimate
    // (accelBias fields used temporarily to pass accel to motion)
    sState.accelBiasX = ax;
    sState.accelBiasY = ay;
    sState.accelBiasZ = az;

    // Motion estimate
    uint32_t motionNowUs = micros();
    float motionDt = 0.01f;
    if (sLastMotionUs != 0) {
        motionDt = (motionNowUs - sLastMotionUs) * 1e-6f;
        if (motionDt < 0.001f) motionDt = 0.001f;
        if (motionDt > 0.05f)  motionDt = 0.05f;
    }
    sLastMotionUs = motionNowUs;
    updateMotionEstimate(motionDt);
}

const FusionState& fusion_get_state() { return sState; }
