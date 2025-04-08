/**
 * Telemetry Serializer
 *
 * Handles serialization and deserialization of telemetry data for transmission
 */

#ifndef TELEMETRY_SERIALIZER_H
#define TELEMETRY_SERIALIZER_H

#include <Arduino.h>
#include <vector>

// Telemetry packet structure
struct TelemetryPacket {
    uint32_t timestamp;            // Timestamp in milliseconds
    uint8_t rocketState;           // Current state of the rocket
    float altitude;                // Altitude in meters
    float verticalSpeed;           // Vertical speed in m/s
    float acceleration;            // Total acceleration in m/s²
    float temperature;             // Temperature in °C
    float pressure;                // Pressure in hPa
    float batteryVoltage;          // Battery voltage in V
    uint8_t gpsSatellites;         // Number of GPS satellites in use
    float gpsLatitude;             // GPS latitude in degrees
    float gpsLongitude;            // GPS longitude in degrees
    float gpsAltitude;             // GPS altitude in meters
    uint8_t sensorStatus;          // Bit field of sensor statuses
    uint8_t flags;                 // Bit field of flags (parachute deployed, etc.)

    // Flag bit definitions
    static const uint8_t FLAG_PARACHUTE_DEPLOYED = 0x01;
    static const uint8_t FLAG_APOGEE_DETECTED = 0x02;
    static const uint8_t FLAG_LANDED = 0x04;
    static const uint8_t FLAG_LOW_BATTERY = 0x08;
    static const uint8_t FLAG_ERROR_CONDITION = 0x10;
};

class TelemetrySerializer {
public:
    TelemetrySerializer() = default;

    // Serialize telemetry data to a byte array
    std::vector<uint8_t> serialize(const TelemetryPacket& packet);

    // Deserialize telemetry data from a byte array
    TelemetryPacket deserialize(const uint8_t* data, size_t length);

    // Helper method to create a telemetry packet with current sensor readings
    static TelemetryPacket createPacket(
            uint8_t rocketState,
            float altitude,
            float verticalSpeed,
            float acceleration,
            float temperature,
            float pressure,
            float batteryVoltage,
            uint8_t gpsSatellites,
            float gpsLatitude,
            float gpsLongitude,
            float gpsAltitude,
            uint8_t sensorStatus,
            uint8_t flags
    );

private:
    // Helper methods for serialization
    void writeUint8(std::vector<uint8_t>& buffer, uint8_t value);
    void writeUint16(std::vector<uint8_t>& buffer, uint16_t value);
    void writeUint32(std::vector<uint8_t>& buffer, uint32_t value);
    void writeFloat(std::vector<uint8_t>& buffer, float value);

    // Helper methods for deserialization
    uint8_t readUint8(const uint8_t* data, size_t& offset);
    uint16_t readUint16(const uint8_t* data, size_t& offset);
    uint32_t readUint32(const uint8_t* data, size_t& offset);
    float readFloat(const uint8_t* data, size_t& offset);
};

#endif // TELEMETRY_SERIALIZER_H