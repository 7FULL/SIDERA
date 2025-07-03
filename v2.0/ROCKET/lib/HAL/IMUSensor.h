#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "Sensor.h"

struct AccelerometerData {
    float x;            // X-axis acceleration in m/s²
    float y;            // Y-axis acceleration in m/s²
    float z;            // Z-axis acceleration in m/s²
    float magnitude;    // Total acceleration magnitude
};

struct GyroscopeData {
    float x;            // X-axis rotation rate in degrees/s
    float y;            // Y-axis rotation rate in degrees/s
    float z;            // Z-axis rotation rate in degrees/s
};

class IMUSensor : public Sensor {
public:
    virtual ~IMUSensor() = default;

    // Get accelerometer data
    virtual AccelerometerData getAccelerometerData() = 0;

    // Get gyroscope data (if available)
    virtual GyroscopeData getGyroscopeData() = 0;

    // Check if gyroscope is available
    virtual bool hasGyroscope() const = 0;

    // Calibrate the sensor
    virtual SensorStatus calibrate() = 0;

    // Set the range for the accelerometer in g (if supported)
    virtual bool setAccelerometerRange(float range) = 0;

    // Get the current accelerometer range in g
    virtual float getAccelerometerRange() const = 0;
};

#endif