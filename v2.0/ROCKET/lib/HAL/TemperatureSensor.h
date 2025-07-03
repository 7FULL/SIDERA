#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "Sensor.h"

class TemperatureSensor : public Sensor {
public:
    virtual ~TemperatureSensor() = default;

    // Get temperature in Celsius
    virtual float getTemperature() = 0;

    // Set temperature offset for calibration
    virtual void setTemperatureOffset(float offset) = 0;

    // Get the temperature offset
    virtual float getTemperatureOffset() const = 0;

protected:
    float temperatureOffset = 0.0f;
};

#endif