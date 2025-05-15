/**
 * Temperature Sensor Manager
 *
 * Manages multiple temperature sensors and provides fusion capabilities
 */

#ifndef TEMPERATURE_SENSOR_MANAGER_H
#define TEMPERATURE_SENSOR_MANAGER_H

#include <vector>
#include "../TemperatureSensor.h"

class TemperatureSensorManager {
public:
    TemperatureSensorManager();
    ~TemperatureSensorManager();

    // Add a sensor to the manager
    void addSensor(TemperatureSensor* sensor);

    // Initialize all sensors
    bool begin();

    // Update all sensor readings
    void update();

    // Get the best available temperature (using fusion if possible)
    float getTemperature();

    // Get the number of operational sensors
    int getOperationalSensorCount();

    // Set temperature offset for all sensors
    void setGlobalTemperatureOffset(float offset);

private:
    std::vector<TemperatureSensor*> sensors;
    bool initialized = false;
    float globalTemperatureOffset = 0.0f;
};

#endif // TEMPERATURE_SENSOR_MANAGER_H