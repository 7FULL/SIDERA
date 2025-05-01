/**
 * Sensor Fusion System
 *
 * Integrates multiple fusion algorithms for a complete state estimation
 */

#ifndef SENSOR_FUSION_SYSTEM_H
#define SENSOR_FUSION_SYSTEM_H

#include "SensorFusionBase.h"
#include "ComplementaryFilter.h"
#include "KalmanFilter.h"
#include "ApogeeDetector.h"
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include "States.h"

// Structure to hold all fused flight data
struct FusedFlightData {
    // Position and motion
    float altitude;         // Current altitude (m)
    float verticalSpeed;    // Vertical speed (m/s)
    float verticalAccel;    // Vertical acceleration (m/s²)

    // Orientation
    float roll;             // Roll angle (degrees)
    float pitch;            // Pitch angle (degrees)
    float yaw;              // Yaw angle (degrees)

    // Flight events
    bool apogeeDetected;    // Whether apogee has been detected
    float apogeeAltitude;   // Altitude at apogee (m)
    float maxAltitude;      // Maximum recorded altitude (m)

    // System status
    float confidence;       // Overall confidence in the data (0-1)
};

class SensorFusionSystem {
public:
    SensorFusionSystem(
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            StorageManager* storageManager = nullptr
    );
    ~SensorFusionSystem();

    // Initialize the fusion system
    bool begin();

    // Update all fusion algorithms
    void update();

    // Get the latest fused flight data
    FusedFlightData getFusedData() const;

    // Reset the fusion system
    void reset();

    // Specific data access
    float getAltitude() const;
    float getVerticalSpeed() const;
    float getVerticalAcceleration() const;
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

    // Event detection
    bool isApogeeDetected() const;
    float getApogeeAltitude() const;
    float getMaxAltitude() const;

    // Confidence metrics
    float getAltitudeConfidence() const;
    float getOrientationConfidence() const;
    float getOverallConfidence() const;

    void setCurrentState(RocketState state);

private:
    RocketState currentState;

    // Subsystem references
    BarometricSensorManager* baroManager;
    IMUSensorManager* imuManager;
    StorageManager* storageManager;

    // Fusion algorithms
    ComplementaryFilter* orientationFilter;
    KalmanFilter* altitudeFilter;
    ApogeeDetector* apogeeDetector;

    // Latest fused data
    FusedFlightData fusedData;

    // Time tracking
    unsigned long lastUpdateTime;
    unsigned long lastLogTime;

    // Internal methods
    void updateAltitudeEstimation();
    void updateOrientationEstimation();
    void detectFlightEvents();
    void calculateConfidence();
    void logFusionData();

    // Vertical acceleration calculation
    float lastVerticalSpeed;
    float verticalAcceleration;
};

#endif // SENSOR_FUSION_SYSTEM_H