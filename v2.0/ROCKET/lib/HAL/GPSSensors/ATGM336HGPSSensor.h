#ifndef ATGM336H_GPS_SENSOR_H
#define ATGM336H_GPS_SENSOR_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "../GPSSensor.h"

class ATGM336HGPSSensor : public GPSSensor {
public:
    ATGM336HGPSSensor(HardwareSerial& serial, int8_t standbyPin = -1, int8_t resetPin = -1);
    ~ATGM336HGPSSensor() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;
    bool sendCommand(const char* cmd);
    bool setUpdateRate(int rateHz); // 1, 2, 5, or 10 Hz

    // Implement GPSSensor interface
    GPSData getGPSData() override;
    bool hasPositionFix() override;
    unsigned long getFixAge() override;
    uint8_t getSatelliteCount() override;
    bool enableLowPowerMode() override;
    bool disableLowPowerMode() override;
    bool reset() override;

private:
    HardwareSerial& serial;
    TinyGPSPlus gps;
    int8_t standbyPin;
    int8_t resetPin;
    bool lowPowerMode;

    bool gpsInitialized;
    GPSData gpsData;
    unsigned long lastDataTime;

    // Debug function
    void displayDebugInfo();
    String getHDOPDescription(float hdopValue) const;
};

#endif