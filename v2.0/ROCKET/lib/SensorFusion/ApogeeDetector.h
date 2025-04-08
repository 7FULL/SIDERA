/**
 * Apogee Detector
 *
 * Advanced algorithm to detect rocket apogee with high confidence
 */

#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include "SensorFusionBase.h"
#include "KalmanFilter.h"
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include <deque>

class ApogeeDetector : public SensorFusionBase {
public:
    ApogeeDetector(BarometricSensorManager* baroManager, IMUSensorManager* imuManager, KalmanFilter* altitudeFilter);
    ~ApogeeDetector() override = default;

    // Implement SensorFusionBase interface
    bool begin() override;
    void update() override;
    void reset() override;
    float getConfidence() const override;

    // Apogee detection status
    bool isApogeeDetected() const;
    float getApogeeAltitude() const;
    float getMaxAltitude() const;
    unsigned long getApogeeTime() const;

    // Configure detector parameters
    void setDetectionThreshold(float threshold);
    void setConfirmationSamples(int samples);
    void setMinimumAltitude(float minAltitude);

private:
    BarometricSensorManager* baroManager;
    IMUSensorManager* imuManager;
    KalmanFilter* altitudeFilter;

    // Detection parameters
    float detectionThreshold;
    int confirmationSamples;
    float minimumAltitude;

    // Detection state
    bool apogeeDetected;
    float apogeeAltitude;
    float maxAltitude;
    unsigned long apogeeTime;
    int descentCount;

    // Data history
    std::deque<float> altitudeHistory;
    std::deque<float> velocityHistory;

    // Time tracking
    unsigned long lastUpdateTime;

    // Confidence in detection
    float confidence;

    // Internal methods
    bool detectApogeeBasic();
    bool detectApogeeAdvanced();
    float computeAltitudeDerivative();
    void updateAltitudeHistory(float altitude);
    void updateVelocityHistory(float velocity);
};

#endif // APOGEE_DETECTOR_H