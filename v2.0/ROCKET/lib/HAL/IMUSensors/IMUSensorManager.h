/**
 * IMU Sensor Manager
 *
 * Manages multiple IMU sensors and provides fusion capabilities
 */

#ifndef IMU_SENSOR_MANAGER_H
#define IMU_SENSOR_MANAGER_H

#include <vector>
#include "../IMUSensor.h"

class IMUSensorManager {
public:
    IMUSensorManager();
    ~IMUSensorManager();

    // Add a sensor to the manager
    void addSensor(IMUSensor* sensor);

    // Initialize all sensors
    bool begin();

    // Update all sensor readings
    void update();

    // Calibrate all sensors
    bool calibrate();

    // Get the best available accelerometer data (using fusion if possible)
    AccelerometerData getAccelerometerData();

    // Get the best available gyroscope data (using fusion if possible)
    GyroscopeData getGyroscopeData();

    // Check if any operational sensor has a gyroscope
    bool hasGyroscope() const;

    // Get the number of operational sensors
    int getOperationalSensorCount();

    // Get the number of operational gyroscopes
    int getOperationalGyroscopeCount();

    // Check if high-G events have been detected
    bool detectHighGEvent(float threshold);

private:
    std::vector<IMUSensor*> sensors;
    AccelerometerData fuseAccelerometerData();
    GyroscopeData fuseGyroscopeData();
    bool initialized = false;

    // Last known acceleration data for each axis (for filtering)
    float lastAccelX = 0.0f;
    float lastAccelY = 0.0f;
    float lastAccelZ = 0.0f;
};

#endif // IMU_SENSOR_MANAGER_H