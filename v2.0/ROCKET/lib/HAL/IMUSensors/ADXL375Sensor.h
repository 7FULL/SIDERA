#ifndef ADXL375_SENSOR_H
#define ADXL375_SENSOR_H

#include <Wire.h>
#include <Adafruit_ADXL375.h>
#include "../IMUSensor.h"
#include "PinDefinitions.h"

class ADXL375Sensor : public IMUSensor {
public:
    ADXL375Sensor(TwoWire& wire = Wire, uint8_t address = AXL_ADDR, int32_t sensorID = 0);
    ~ADXL375Sensor() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;

    // Implement IMUSensor interface
    AccelerometerData getAccelerometerData() override;
    GyroscopeData getGyroscopeData() override;
    bool hasGyroscope() const override;
    SensorStatus calibrate() override;
    bool setAccelerometerRange(float range) override;
    float getAccelerometerRange() const override;

    // ADXL375 specific methods
    bool setDataRate(adxl3xx_dataRate_t dataRate);

private:
    Adafruit_ADXL375 sensor;
    TwoWire& wire;
    int32_t sensorID;
    uint8_t address;
    float accelRange = 200.0f; // Fixed ±200g for ADXL375
    AccelerometerData accelData;
    GyroscopeData dummyGyroData; // ADXL375 has no gyroscope

    // Calibration offsets
    float offsetX = 0.0f;
    float offsetY = 0.0f;
    float offsetZ = 0.0f;
};

#endif