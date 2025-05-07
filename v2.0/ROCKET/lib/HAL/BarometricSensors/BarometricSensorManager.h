/**
 * Barometric Sensor Manager
 *
 * Manages multiple barometric sensors with failover capability
 */

#ifndef BAROMETRIC_SENSOR_MANAGER_H
#define BAROMETRIC_SENSOR_MANAGER_H

#include <vector>
#include "../BarometricSensor.h"

class BarometricSensorManager {
public:
    BarometricSensorManager();
    ~BarometricSensorManager();

    // Add a sensor with priority (lower number = higher priority)
    void addSensor(BarometricSensor* sensor, uint8_t priority = 0);

    // Initialize all sensors
    bool begin();

    // Update all sensor readings
    void update();

    // Set the reference altitude for all sensors
    void setReferenceAltitude(float altitude);

    // Get altitude from the best available sensor
    float getAltitude();

    // Get pressure from the best available sensor
    float getPressure();

    // Get temperature from the best available sensor
    float getTemperature();

    // Get the number of operational sensors
    int getOperationalSensorCount();

    // Get the name of the currently active sensor
    const char* getActiveSensorName();

private:
    struct SensorInfo {
        BarometricSensor* sensor;
        uint8_t priority;
    };

    std::vector<SensorInfo> sensors;
    BarometricSensor* activeSensor = nullptr;
    bool initialized = false;

    // Finds the best operational sensor based on priority
    BarometricSensor* findBestSensor();

    // Updates the active sensor if needed
    void updateActiveSensor();
};

#endif // BAROMETRIC_SENSOR_MANAGER_H