#ifndef BMP388_SENSOR_H
#define BMP388_SENSOR_H

#include <Wire.h>
#include "Adafruit_BMP3XX.h"
#include "../BarometricSensor.h"
#include "PinDefinitions.h"

class BMP388Sensor : public BarometricSensor {
public:
    BMP388Sensor(TwoWire& wire = Wire1, uint8_t address = BMP_ADDR);
    ~BMP388Sensor() override;

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
    Adafruit_BMP3XX sensor;
    TwoWire& wire;
    uint8_t address;
    float temperature;
    float pressure;
    float altitude;
    static constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25;
};

#endif