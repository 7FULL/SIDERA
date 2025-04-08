/**
 * Barometric Sensor Manager
 *
 * Manages multiple barometric sensors and provides fusion capabilities
 */

#ifndef BAROMETRIC_SENSOR_MANAGER_H
#define BAROMETRIC_SENSOR_MANAGER_H

#include <vector>
#include "../BarometricSensor.h"

class BarometricSensorManager {
public:
    BarometricSensorManager();
    ~BarometricSensorManager();

    // Add a sensor to the manager
    void addSensor(BarometricSensor* sensor);

    // Initialize all sensors
    bool begin();

    // Update all sensor readings
    void update();

    // Set the reference altitude for all sensors
    void setReferenceAltitude(float altitude);

    // Get the best available altitude (using fusion if possible)
    float getAltitude();

    // Get the best available pressure (using fusion if possible)
    float getPressure();

    // Get the best available temperature (using fusion if possible)
    float getTemperature();

    // Get the number of operational sensors
    int getOperationalSensorCount();

private:
    std::vector<BarometricSensor*> sensors;
    float fuseAltitudeData();
    float fusePressureData();
    float fuseTemperatureData();
    bool initialized = false;
};

#endif // BAROMETRIC_SENSOR_MANAGER_H