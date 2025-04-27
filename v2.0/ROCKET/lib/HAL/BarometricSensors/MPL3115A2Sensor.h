/**
 * MPL3115A2 Barometric Sensor Implementation
 */

#ifndef MPL3115A2_SENSOR_H
#define MPL3115A2_SENSOR_H

#include <Wire.h>
#include "Adafruit_MPL3115A2.h"
#include "../BarometricSensor.h"

class MPL3115A2Sensor : public BarometricSensor {
public:
    MPL3115A2Sensor(TwoWire& wire = Wire, uint8_t address = 0x60);
    ~MPL3115A2Sensor() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;

    // Implement BarometricSensor interface
    float getAltitude() override;
    float getPressure() override;
    float getTemperature() override;
    void setReferenceAltitude(float altitude) override;
    float getReferenceAltitude() const override;

private:
    Adafruit_MPL3115A2 sensor;
    TwoWire& wire;
    uint8_t address;
    float temperature;
    float pressure;
    float altitude;
};

#endif // MPL3115A2_SENSOR_H