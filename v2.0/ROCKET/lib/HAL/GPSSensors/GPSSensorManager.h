/**
 * GPS Sensor Manager
 *
 * Manages multiple GPS sensors with automatic failover
 */

#ifndef GPS_SENSOR_MANAGER_H
#define GPS_SENSOR_MANAGER_H

#include <vector>
#include "../GPSSensor.h"

class GPSSensorManager {
public:
    GPSSensorManager();
    ~GPSSensorManager();

    // Add a GPS sensor with priority (lower number = higher priority)
    void addSensor(GPSSensor* sensor, uint8_t priority = 0);

    // Initialize all sensors
    bool begin();

    // Update all sensor readings
    void update();

    // Get the best available GPS data
    GPSData getGPSData();

    // Check if any sensor has a position fix
    bool hasPositionFix();

    // Get the age of the best position fix
    unsigned long getFixAge();

    // Get the number of satellites from the best sensor
    uint8_t getSatelliteCount();

    // Enable low power mode for all sensors
    bool enableLowPowerMode();

    // Disable low power mode for all sensors
    bool disableLowPowerMode();

    // Get the name of the currently active sensor
    const char* getActiveSensorName();

    // Get the number of operational sensors
    int getOperationalSensorCount();

private:
    struct GPSSensorInfo {
        GPSSensor* sensor;
        uint8_t priority;
    };

    std::vector<GPSSensorInfo> sensors;
    bool initialized = false;
    GPSSensor* activeSensor = nullptr;
    GPSData lastValidData;

    void updateActiveSensor();
    GPSSensor* findBestSensor();
};

#endif // GPS_SENSOR_MANAGER_H