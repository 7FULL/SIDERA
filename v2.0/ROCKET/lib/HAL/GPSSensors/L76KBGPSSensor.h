#ifndef L76KB_GPS_SENSOR_H
#define L76KB_GPS_SENSOR_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "../GPSSensor.h"

class L76KBGPSSensor : public GPSSensor {
public:
    L76KBGPSSensor(HardwareSerial& serial, int8_t standbyPin = -1, int8_t resetPin = -1);
    ~L76KBGPSSensor() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;

    // Implement GPSSensor interface
    GPSData getGPSData() override;
    bool hasPositionFix() override;
    unsigned long getFixAge() override;
    uint8_t getSatelliteCount() override;
    bool enableLowPowerMode() override;
    bool disableLowPowerMode() override;
    bool reset() override;

    // L76KB specific methods
    bool sendCommand(const char* cmd);
    bool setUpdateRate(int rateHz); // 1, 2, 5, or 10 Hz

private:
    HardwareSerial& serial;
    TinyGPSPlus gps;
    int8_t standbyPin;
    int8_t resetPin;
    bool lowPowerMode;

    GPSData gpsData;
    bool gpsInitialized;
    unsigned long lastDataTime;

    // Debug function
    void displayDebugInfo();
    String getHDOPDescription(float hdopValue) const;
};

#endif