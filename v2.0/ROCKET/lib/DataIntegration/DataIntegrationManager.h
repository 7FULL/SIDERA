/**
 * Data Integration Manager
 *
 * Centralizes sensor data collection and provides detection algorithms
 */

#ifndef DATA_INTEGRATION_MANAGER_H
#define DATA_INTEGRATION_MANAGER_H

#include <Arduino.h>
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"
#include "../HAL/TemperatureSensors/TemperatureSensorManager.h"
#include "../PowerManagement/PowerManager.h"
#include "../../include/States.h"
#include "Config.h"

// Define FlightData structure for integrated sensor data
struct FlightData {
    float altitude;
    float verticalSpeed;
    float verticalAccel;
    float temperature;
    float pressure;
    AccelerometerData accelData;
    GyroscopeData gyroData;
    GPSData gpsData;
    float batteryVoltage;
    float confidence;
    bool apogeeDetected;
    bool landingDetected;
    uint8_t gpsSatellites;
    uint8_t status;
    unsigned long timestamp;
    RocketState state;
};

class DataIntegrationManager {
public:
    DataIntegrationManager(
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            TemperatureSensorManager* tempManager = nullptr,
            PowerManager* powerManager = nullptr
    );

    // Initialize the manager
    bool begin();

    // Update all data
    void update();

    // Set the current rocket state (for reference)
    void setCurrentState(RocketState state);

    // Get the integrated flight data
    FlightData getFlightData();

    // Check if apogee was detected
    bool isApogeeDetected();

    // Check if landing was detected
    bool isLandingDetected();

private:
    // Sensor manager references
    BarometricSensorManager* baroManager;
    IMUSensorManager* imuManager;
    GPSSensorManager* gpsManager;
    TemperatureSensorManager* tempManager;
    PowerManager* powerManager;

    // Current rocket state
    RocketState currentState;

    // Integrated flight data
    FlightData flightData;

    // Apogee detection
    bool apogeeDetected;
    float maxAltitude;
    int descentCount;

    // Landing detection
    bool landingDetected;
    float landingAltitude;
    unsigned long stableStartTime;
    bool stableAltitudeDetected;

    // Vertical speed calculation
    float lastAltitude;
    unsigned long lastAltitudeTime;

    // Helper methods for detection algorithms
    void detectApogee();
    void detectLanding();
    void calculateVerticalSpeed();
};

#endif // DATA_INTEGRATION_MANAGER_H