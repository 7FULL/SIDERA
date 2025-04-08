/**
 * Complementary Filter Implementation
 */

#include "ComplementaryFilter.h"
#include <math.h>

ComplementaryFilter::ComplementaryFilter(IMUSensorManager* imuManager)
        : imuManager(imuManager),
          roll(0.0f),
          pitch(0.0f),
          yaw(0.0f),
          alpha(0.01f),      // Default low-pass filter coefficient
          gyroTrust(0.98f),  // Default gyro trust level (98% gyro, 2% accel)
          lastUpdateTime(0),
          confidence(0.0f) {
}

bool ComplementaryFilter::begin() {
    if (!imuManager) {
        return false;
    }

    // Initialize with accelerometer data to get initial orientation
    AccelerometerData accelData = imuManager->getAccelerometerData();
    updateFromAccelerometer(accelData);

    lastUpdateTime = millis();
    confidence = 0.5f;  // Start with moderate confidence

    return true;
}

void ComplementaryFilter::update() {
    if (!imuManager) {
        return;
    }

    // Calculate time delta
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;  // in seconds
    lastUpdateTime = currentTime;

    // Sanity check on dt
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;  // Default to 10ms if time delta is invalid
    }

    // Get sensor data
    AccelerometerData accelData = imuManager->getAccelerometerData();

    // Calculate accelerometer-based orientation
    float accelRoll = 0.0f;
    float accelPitch = 0.0f;

    // Only calculate accel-based orientation if we have valid data
    if (accelData.magnitude > 0.5f && accelData.magnitude < 30.0f) {
        // Calculate roll and pitch from accelerometer
        // Roll (rotation around X-axis)
        accelRoll = atan2(accelData.y, accelData.z) * 180.0f / M_PI;

        // Pitch (rotation around Y-axis)
        accelPitch = atan2(-accelData.x, sqrt(accelData.y * accelData.y + accelData.z * accelData.z)) * 180.0f / M_PI;
    }

    // Update orientation from gyroscope
    if (imuManager->hasGyroscope()) {
        GyroscopeData gyroData = imuManager->getGyroscopeData();
        updateFromGyroscope(gyroData, dt);
    }

    // Combine orientations using complementary filter
    combineOrientations(accelRoll, accelPitch, dt);

    // Update confidence based on accelerometer data quality
    float accelMagnitudeError = abs(accelData.magnitude - 9.81f) / 9.81f;
    float accelConfidence = 1.0f - constrainFloat(accelMagnitudeError, 0.0f, 1.0f);

    // Update overall confidence (weighted average with previous confidence)
    confidence = lowPassFilter(accelConfidence, confidence, 0.1f);
}

void ComplementaryFilter::reset() {
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    lastUpdateTime = millis();
    confidence = 0.0f;
}

float ComplementaryFilter::getConfidence() const {
    return confidence;
}

float ComplementaryFilter::getRoll() const {
    return roll;
}

float ComplementaryFilter::getPitch() const {
    return pitch;
}

float ComplementaryFilter::getYaw() const {
    return yaw;
}

void ComplementaryFilter::setFilterCoefficient(float alpha) {
    this->alpha = constrainFloat(alpha, 0.0f, 1.0f);
}

void ComplementaryFilter::setGyroTrust(float gyroTrust) {
    this->gyroTrust = constrainFloat(gyroTrust, 0.0f, 1.0f);
}

void ComplementaryFilter::updateFromAccelerometer(const AccelerometerData& accelData) {
    // Only update if we have valid acceleration data
    if (accelData.magnitude < 0.1f) {
        return;  // Too small to be valid
    }

    // Calculate roll (rotation around X-axis)
    float newRoll = atan2(accelData.y, accelData.z) * 180.0f / M_PI;

    // Calculate pitch (rotation around Y-axis)
    float newPitch = atan2(-accelData.x, sqrt(accelData.y * accelData.y + accelData.z * accelData.z)) * 180.0f / M_PI;

    // Apply low-pass filter to reduce noise
    roll = lowPassFilter(newRoll, roll, alpha);
    pitch = lowPassFilter(newPitch, pitch, alpha);

    // Note: Yaw cannot be determined from accelerometer alone
}

void ComplementaryFilter::updateFromGyroscope(const GyroscopeData& gyroData, float dt) {
    // Update orientation based on gyroscope data and time delta
    roll += gyroData.x * dt;
    pitch += gyroData.y * dt;
    yaw += gyroData.z * dt;

    // Normalize yaw to 0-360 degrees
    while (yaw < 0.0f) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
}

void ComplementaryFilter::combineOrientations(float accelRoll, float accelPitch, float dt) {
    // Don't use complementary filter if dt is too large (would give too much weight to gyro)
    if (dt > 0.1f) {
        return;
    }

    // Apply complementary filter: combine gyro and accelerometer data
    // High-pass filter on gyro (keeps high-frequency changes) and
    // low-pass filter on accelerometer (keeps steady-state value)
    roll = gyroTrust * roll + (1.0f - gyroTrust) * accelRoll;
    pitch = gyroTrust * pitch + (1.0f - gyroTrust) * accelPitch;

    // Yaw is based solely on gyroscope as accelerometer cannot determine it
}