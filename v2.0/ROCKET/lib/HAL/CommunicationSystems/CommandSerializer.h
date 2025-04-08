/**
 * Command Serializer
 *
 * Handles serialization and deserialization of command data for transmission
 */

#ifndef COMMAND_SERIALIZER_H
#define COMMAND_SERIALIZER_H

#include <Arduino.h>
#include <vector>

// Command types
enum class CommandType : uint8_t {
    PING = 1,
    DEPLOY_PARACHUTE = 2,
    ABORT = 3,
    REBOOT = 4,
    WAKE_UP = 5,
    LAUNCH = 6,
    SET_PARAMETER = 7,
    GET_PARAMETER = 8,
    ENTER_LOW_POWER = 9,
    EXIT_LOW_POWER = 10
};

// Command packet structure
struct CommandPacket {
    CommandType commandType;   // Type of command
    uint32_t timestamp;        // Timestamp in milliseconds
    uint16_t parameter1;       // Command-specific parameter 1
    uint16_t parameter2;       // Command-specific parameter 2
    std::vector<uint8_t> data; // Optional data payload
};

class CommandSerializer {
public:
    CommandSerializer() = default;

    // Serialize command data to a byte array
    std::vector<uint8_t> serialize(const CommandPacket& packet);

    // Deserialize command data from a byte array
    CommandPacket deserialize(const uint8_t* data, size_t length);

    // Helper method to create a command packet
    static CommandPacket createPacket(
            CommandType commandType,
            uint16_t parameter1 = 0,
            uint16_t parameter2 = 0,
            const std::vector<uint8_t>& data = std::vector<uint8_t>()
    );

private:
    // Helper methods for serialization
    void writeUint8(std::vector<uint8_t>& buffer, uint8_t value);
    void writeUint16(std::vector<uint8_t>& buffer, uint16_t value);
    void writeUint32(std::vector<uint8_t>& buffer, uint32_t value);

    // Helper methods for deserialization
    uint8_t readUint8(const uint8_t* data, size_t& offset);
    uint16_t readUint16(const uint8_t* data, size_t& offset);
    uint32_t readUint32(const uint8_t* data, size_t& offset);
};

#endif // COMMAND_SERIALIZER_H