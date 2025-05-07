/**
 * IMU Sensor Manager
 *
 * Manages multiple IMU sensors with failover capability
 */

#ifndef IMU_SENSOR_MANAGER_H
#define IMU_SENSOR_MANAGER_H

#include <vector>
#include "../IMUSensor.h"

class IMUSensorManager {
public:
    IMUSensorManager();
    ~IMUSensorManager();

    // Add a sensor with priority (lower number = higher priority)
    void addSensor(IMUSensor* sensor, uint8_t priority = 0);

    // Initialize all sensors
    bool begin();

    // Update all sensor readings
    void update();

    // Calibrate all sensors
    bool calibrate();

    // Get accelerometer data from the best available sensor
    AccelerometerData getAccelerometerData();

    // Get gyroscope data from the best available sensor with a gyroscope
    GyroscopeData getGyroscopeData();

    // Check if any operational sensor has a gyroscope
    bool hasGyroscope() const;

    // Get the number of operational sensors
    int getOperationalSensorCount();

    // Get the number of operational gyroscopes
    int getOperationalGyroscopeCount();

    // Check if high-G events have been detected
    bool detectHighGEvent(float threshold);

    // Get the name of the currently active sensor
    const char* getActiveSensorName();

    // Get the name of the currently active gyroscope sensor
    const char* getActiveGyroscopeSensorName();

private:
    struct SensorInfo {
        IMUSensor* sensor;
        uint8_t priority;
    };

    std::vector<SensorInfo> sensors;
    IMUSensor* activeAccelSensor = nullptr;
    IMUSensor* activeGyroSensor = nullptr;
    bool initialized = false;

    // Finds the best operational accelerometer sensor
    IMUSensor* findBestAccelSensor();

    // Finds the best operational gyroscope sensor
    IMUSensor* findBestGyroSensor();

    // Updates the active sensors if needed
    void updateActiveSensors();
};

#endif // IMU_SENSOR_MANAGER_H