/**
 * Complementary Filter
 *
 * Combines accelerometer and gyroscope data to estimate orientation
 */

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "SensorFusionBase.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"

class ComplementaryFilter : public SensorFusionBase {
public:
    ComplementaryFilter(IMUSensorManager* imuManager);
    ~ComplementaryFilter() override = default;

    // Implement SensorFusionBase interface
    bool begin() override;
    void update() override;
    void reset() override;
    float getConfidence() const override;

    // Orientation getters
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

    // Set filter parameters
    void setFilterCoefficient(float alpha);
    void setGyroTrust(float gyroTrust);

private:
    IMUSensorManager* imuManager;

    // Orientation in degrees
    float roll;
    float pitch;
    float yaw;

    // Filter parameters
    float alpha;
    float gyroTrust;

    // Time tracking
    unsigned long lastUpdateTime;

    // Confidence in estimation
    float confidence;

    // Internal methods
    void updateFromAccelerometer(const AccelerometerData& accelData);
    void updateFromGyroscope(const GyroscopeData& gyroData, float dt);
    void combineOrientations(float accelRoll, float accelPitch, float dt);
};

#endif // COMPLEMENTARY_FILTER_H