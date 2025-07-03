#ifndef STORAGE_SYSTEM_H
#define STORAGE_SYSTEM_H

#include "Sensor.h" // Reusamos la interfaz Sensor para mantener coherencia
#include <Stream.h>

// Log entry structure
struct LogEntry {
    uint32_t timestamp;   // Timestamp in milliseconds
    uint8_t logLevel;     // Log severity level
    uint8_t subsystem;    // Source subsystem
    char message[64];     // Log message
};

// Telemetry entry structure (compact version for storage)
struct StoredTelemetry {
    uint32_t timestamp;   // Timestamp in milliseconds
    uint8_t state;        // Rocket state
    int32_t altitude;     // Altitude in mm (scaled to save space)
    int16_t vertSpeed;    // Vertical speed in cm/s (scaled)
    int16_t accelX;       // Acceleration X in 0.01 g (scaled)
    int16_t accelY;       // Acceleration Y in 0.01 g (scaled)
    int16_t accelZ;       // Acceleration Z in 0.01 g (scaled)
    int16_t gyroX;        // Gyro X in 0.1 deg/s (scaled)
    int16_t gyroY;        // Gyro Y in 0.1 deg/s (scaled)
    int16_t gyroZ;        // Gyro Z in 0.1 deg/s (scaled)
    int16_t temperature;  // Temperature in 0.1°C (scaled)
    uint16_t pressure;    // Pressure in 0.1 hPa (scaled)
    uint16_t batteryMv;   // Battery voltage in mV
    int32_t gpsLat;       // GPS latitude in 10^-7 degrees (scaled)
    int32_t gpsLon;       // GPS longitude in 10^-7 degrees (scaled)
    uint16_t gpsAlt;      // GPS altitude in meters
    uint8_t gpsSats;      // Number of GPS satellites
    uint8_t flags;        // Status flags
};

// Log levels
enum class LogLevel : uint8_t {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4
};

// Subsystem identifiers
enum class Subsystem : uint8_t {
    SYSTEM = 0,
    SENSORS = 1,
    BAROMETER = 2,
    IMU = 3,
    GPS = 4,
    COMMUNICATION = 5,
    STORAGE = 6,
    STATE_MACHINE = 7,
    EVENTS = 8,
    PARACHUTE = 9
};

class StorageSystem : public Sensor {
public:
    virtual ~StorageSystem() = default;

    // Initialize the storage system
    virtual SensorStatus begin() override = 0;

    // Log a message
    virtual bool logMessage(LogLevel level, Subsystem subsystem, const char* message) = 0;

    // Store telemetry data
    virtual bool storeTelemetry(const StoredTelemetry& telemetry) = 0;

    // Flush any buffered data to storage
    virtual bool flush() = 0;

    // Get total storage capacity in bytes
    virtual uint32_t getTotalCapacity() = 0;

    // Get available storage space in bytes
    virtual uint32_t getAvailableSpace() = 0;

    // Get the path or identifier of the current log file
    virtual const char* getCurrentLogIdentifier() = 0;

    // Export all stored data to a stream (e.g., a file on SD card)
    virtual bool exportDataTo(Stream& outputStream) = 0;

    // Clear all stored data
    virtual bool clearAllData() = 0;
};

#endif