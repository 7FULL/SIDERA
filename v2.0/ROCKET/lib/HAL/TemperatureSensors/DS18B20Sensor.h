#ifndef DS18B20_SENSOR_H
#define DS18B20_SENSOR_H

#include <OneWire.h>
#include <DallasTemperature.h>
#include "../TemperatureSensor.h"

class DS18B20Sensor : public TemperatureSensor {
public:
    DS18B20Sensor(uint8_t pin, uint8_t resolution = 12);
    ~DS18B20Sensor() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;

    // Implement TemperatureSensor interface
    float getTemperature() override;
    void setTemperatureOffset(float offset) override;
    float getTemperatureOffset() const override;

    // DS18B20 specific methods
    void setResolution(uint8_t resolution);
    uint8_t getResolution() const;
    DeviceAddress* getDeviceAddress();
    void setAsyncResolution(uint8_t resolution);

private:
    uint8_t pin;
    uint8_t resolution;
    float temperature;
    DeviceAddress deviceAddress;
    bool deviceFound;

    OneWire oneWire;
    DallasTemperature sensors;

    bool conversionInProgress = false;
    unsigned long conversionStartTime = 0;
    unsigned long conversionDelay = 750; // Default for 12-bit
};

#endif