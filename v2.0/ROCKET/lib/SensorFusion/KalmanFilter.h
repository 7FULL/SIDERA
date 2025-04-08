/**
 * Kalman Filter
 *
 * Provides optimal state estimation for altitude and vertical velocity
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "SensorFusionBase.h"

// Simple 2-state Kalman filter (position, velocity)
class KalmanFilter : public SensorFusionBase {
public:
    KalmanFilter();
    ~KalmanFilter() override = default;

    // Implement SensorFusionBase interface
    bool begin() override;
    void update() override;
    void reset() override;
    float getConfidence() const override;

    // Set initial state
    void setState(float position, float velocity);

    // Get state estimates
    float getPosition() const;
    float getVelocity() const;

    // Update with measurements
    void updateWithPosition(float measurement, float uncertainty);
    void updateWithAcceleration(float measurement, float uncertainty, float dt);

    // Configure filter parameters
    void setProcessNoise(float positionNoise, float velocityNoise);
    void setPredictionFactor(float factor);

private:
    // State vector [position, velocity]
    float stateVector[2];

    // Covariance matrix [2x2]
    float covarianceMatrix[2][2];

    // Process noise covariance
    float processNoise[2];

    // Prediction factor (for non-linear adjustments)
    float predictionFactor;

    // Time tracking
    unsigned long lastUpdateTime;

    // Confidence in state estimate
    float confidence;

    // Internal methods
    void predict(float dt);
    void updateMeasurement(float measurement, float uncertainty, int measurementIndex);
};

#endif // KALMAN_FILTER_H