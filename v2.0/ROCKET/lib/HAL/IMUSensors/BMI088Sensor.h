#ifndef BMI088_SENSOR_H
#define BMI088_SENSOR_H
#include "PinDefinitions.h"

#include <Wire.h>
#include <BMI088.h>
#include "../IMUSensor.h"

class BMI088Sensor : public IMUSensor {
public:
    BMI088Sensor(TwoWire& wire = Wire, uint8_t gyroAddress = BMIO_GYR_ADDR, uint8_t accelAddress = BMIO_ACCEL_ADDR);
    ~BMI088Sensor() override;

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

private:
    Bmi088Accel accel;
    Bmi088Gyro gyro;
    TwoWire& wire;

    float accelOffsetX = 0.0f;
    float accelOffsetY = 0.0f;
    float accelOffsetZ = 0.0f;

    float accelRange = 3.0f; // ±3g by default
    AccelerometerData accelData;
    GyroscopeData gyroData;

    bool accelInitialized = false;
    bool gyroInitialized = false;
};

#endif