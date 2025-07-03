#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include "Sensor.h"

struct GPSData {
    float latitude;      // Latitude in degrees (negative for South)
    float longitude;     // Longitude in degrees (negative for West)
    float altitude;      // Altitude in meters above sea level
    float speed;         // Speed in m/s
    float course;        // Course in degrees (0-359.99)
    uint8_t satellites;  // Number of satellites in use
    float hdop;          // Horizontal dilution of precision
    uint16_t year;       // Year (2000+)
    uint8_t month;       // Month (1-12)
    uint8_t day;         // Day (1-31)
    uint8_t hour;        // Hour (0-23)
    uint8_t minute;      // Minute (0-59)
    uint8_t second;      // Second (0-59)
    bool valid;          // Whether the GPS data is valid
    unsigned long age;   // Age of the GPS data in milliseconds
};

class GPSSensor : public Sensor {
public:
    virtual ~GPSSensor() = default;

    // Get GPS data
    virtual GPSData getGPSData() = 0;

    // Check if position fix is available
    virtual bool hasPositionFix() = 0;

    // Get time since last position fix in milliseconds
    virtual unsigned long getFixAge() = 0;

    // Get the number of satellites currently in use
    virtual uint8_t getSatelliteCount() = 0;

    // Enter low power mode (if supported)
    virtual bool enableLowPowerMode() = 0;

    // Exit low power mode (if supported)
    virtual bool disableLowPowerMode() = 0;

    // Reset the GPS module (if supported)
    virtual bool reset() = 0;
};

#endif