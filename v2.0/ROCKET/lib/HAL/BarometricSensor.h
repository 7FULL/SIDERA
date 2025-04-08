/**
 * Barometric Sensor Interface
 *
 * Base class for all barometric/pressure sensors
 */

#ifndef BAROMETRIC_SENSOR_H
#define BAROMETRIC_SENSOR_H

#include "Sensor.h"

class BarometricSensor : public Sensor {
public:
    virtual ~BarometricSensor() = default;

    // Get altitude in meters
    virtual float getAltitude() = 0;

    // Get pressure in Pascals
    virtual float getPressure() = 0;

    // Get temperature in Celsius
    virtual float getTemperature() = 0;

    // Set reference altitude (ground level)
    virtual void setReferenceAltitude(float altitude) = 0;

    // Get the reference altitude
    virtual float getReferenceAltitude() const = 0;

protected:
    float referenceAltitude = 0.0f;
};

#endif // BAROMETRIC_SENSOR_H