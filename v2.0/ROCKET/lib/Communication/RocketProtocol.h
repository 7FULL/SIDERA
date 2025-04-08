#ifndef ROCKET_PROTOCOL_H
#define ROCKET_PROTOCOL_H

#include <Arduino.h>
#include <vector>

// Command types for ground station to rocket
enum class CommandCode : uint8_t {
    PING = 0x01,
    GET_STATUS = 0x02,
    GET_TELEMETRY = 0x03,
    ARM_ROCKET = 0x10,
    DISARM_ROCKET = 0x11,
    START_COUNTDOWN = 0x12,
    ABORT_COUNTDOWN = 0x13,
    FORCE_DEPLOY_PARACHUTE = 0x20,
    CALIBRATE_SENSORS = 0x30,
    RUN_DIAGNOSTICS = 0x31,
    SET_PARAMETER = 0x40,
    GET_PARAMETER = 0x41,
    ENTER_LOW_POWER = 0x50,
    EXIT_LOW_POWER = 0x51,
    RESET_SYSTEM = 0xF0
};

// Response types for rocket to ground station
enum class ResponseCode : uint8_t {
    ACK = 0x01,
    NACK = 0x02,
    TELEMETRY_DATA = 0x10,
    STATUS_DATA = 0x11,
    PARAMETER_VALUE = 0x20,
    DIAGNOSTIC_RESULT = 0x30,
    EVENT_NOTIFICATION = 0x40,
    ERROR_MESSAGE = 0xE0
};

// Rocket status codes
enum class RocketStatusCode : uint8_t {
    INITIALIZING = 0x01,
    IDLE = 0x02,
    READY = 0x03,
    ARMED = 0x04,
    COUNTDOWN = 0x05,
    POWERED_FLIGHT = 0x10,
    COASTING = 0x11,
    APOGEE = 0x12,
    DESCENT = 0x13,
    PARACHUTE_DEPLOYED = 0x14,
    LANDED = 0x20,
    ERROR = 0xE0
};

// Packet structure for communication
struct ProtocolPacket {
    static const uint16_t HEADER_MAGIC = 0xA55A;  // Magic number for packet identification
    static const uint8_t PROTOCOL_VERSION = 0x01; // Protocol version

    uint16_t header;         // Magic header (HEADER_MAGIC)
    uint8_t version;         // Protocol version
    uint8_t type;            // Command or Response code
    uint16_t sequenceNumber; // Packet sequence number
    uint16_t length;         // Payload length
    uint8_t* payload;        // Payload data
    uint16_t checksum;       // CRC-16 checksum

    // For telemetry packets
    static const uint16_t MAX_PAYLOAD_SIZE = 240;  // Maximum payload size
};

// Parameter types
struct ParameterId {
    static const uint8_t TELEMETRY_RATE = 0x01;
    static const uint8_t SENSOR_UPDATE_RATE = 0x02;
    static const uint8_t LORA_POWER = 0x03;
    static const uint8_t APOGEE_DETECTION_THRESHOLD = 0x10;
    static const uint8_t LANDING_DETECTION_THRESHOLD = 0x11;
    static const uint8_t LOGGING_LEVEL = 0x20;
};

class RocketProtocol {
public:
    // Initialize protocol
    static void initialize();

    // Create a command packet
    static std::vector<uint8_t> createCommandPacket(CommandCode code,
                                                    const uint8_t* payload = nullptr,
                                                    uint16_t length = 0);

    // Create a response packet
    static std::vector<uint8_t> createResponsePacket(ResponseCode code,
                                                     const uint8_t* payload = nullptr,
                                                     uint16_t length = 0,
                                                     uint16_t sequenceNumber = 0);

    // Parse a received packet
    static bool parsePacket(const uint8_t* data, uint16_t length, ProtocolPacket& packet);

    // Calculate CRC-16 checksum
    static uint16_t calculateCrc16(const uint8_t* data, uint16_t length);

    // Verify packet integrity
    static bool verifyPacket(const uint8_t* data, uint16_t length);

    // Get current sequence number
    static uint16_t getCurrentSequence();

private:
    static uint16_t sequenceCounter;  // Tracks packet sequence numbers
};

#endif // ROCKET_PROTOCOL_H