#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// Sensor status values
enum class SensorStatus {
    OK,                 // Sensor is functioning correctly
    NOT_INITIALIZED,    // Sensor has not been initialized
    INITIALIZATION_ERROR, // Failed to initialize
    COMMUNICATION_ERROR,  // CommunicationSystems with sensor failed
    READING_ERROR,      // Error reading data
    CALIBRATION_ERROR,  // Error during calibration
    TIMEOUT            // Operation timed out
};

// Base sensor interface
class Sensor {
public:
    // Virtual destructor for proper cleanup
    virtual ~Sensor() = default;

    // Initialize the sensor
    virtual SensorStatus begin() = 0;

    // Update sensor readings
    virtual SensorStatus update() = 0;

    // Check if sensor is functioning properly
    virtual bool isOperational() = 0;

    // Get the last known status
    virtual SensorStatus getStatus() const = 0;

    // Get sensor name
    virtual const char* getName() const = 0;

    // Get last sensor reading timestamp
    virtual unsigned long getLastReadingTime() const = 0;

protected:
    SensorStatus status = SensorStatus::NOT_INITIALIZED;
    unsigned long lastReadingTime = 0;
};

#endif