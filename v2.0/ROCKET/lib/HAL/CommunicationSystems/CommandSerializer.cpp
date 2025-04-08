/**
 * Command Serializer Implementation
 */

#include "CommandSerializer.h"

std::vector<uint8_t> CommandSerializer::serialize(const CommandPacket& packet) {
    std::vector<uint8_t> buffer;
    buffer.reserve(10 + packet.data.size()); // Base size + data

    // Write command type
    writeUint8(buffer, static_cast<uint8_t>(packet.commandType));

    // Write timestamp
    writeUint32(buffer, packet.timestamp);

    // Write parameters
    writeUint16(buffer, packet.parameter1);
    writeUint16(buffer, packet.parameter2);

    // Write data length
    writeUint16(buffer, packet.data.size());

    // Write data payload
    buffer.insert(buffer.end(), packet.data.begin(), packet.data.end());

    return buffer;
}

CommandPacket CommandSerializer::deserialize(const uint8_t* data, size_t length) {
    CommandPacket packet;
    size_t offset = 0;

    // Read command type
    if (offset + 1 <= length) {
        packet.commandType = static_cast<CommandType>(readUint8(data, offset));
    }

    // Read timestamp
    if (offset + 4 <= length) {
        packet.timestamp = readUint32(data, offset);
    }

    // Read parameters
    if (offset + 2 <= length) {
        packet.parameter1 = readUint16(data, offset);
    }

    if (offset + 2 <= length) {
        packet.parameter2 = readUint16(data, offset);
    }

    // Read data length
    uint16_t dataLength = 0;
    if (offset + 2 <= length) {
        dataLength = readUint16(data, offset);
    }

    // Read data payload
    if (offset + dataLength <= length) {
        packet.data.assign(data + offset, data + offset + dataLength);
        offset += dataLength;
    }

    return packet;
}

CommandPacket CommandSerializer::createPacket(
        CommandType commandType,
        uint16_t parameter1,
        uint16_t parameter2,
        const std::vector<uint8_t>& data
) {
    CommandPacket packet;

    packet.commandType = commandType;
    packet.timestamp = millis();
    packet.parameter1 = parameter1;
    packet.parameter2 = parameter2;
    packet.data = data;

    return packet;
}

void CommandSerializer::writeUint8(std::vector<uint8_t>& buffer, uint8_t value) {
    buffer.push_back(value);
}

void CommandSerializer::writeUint16(std::vector<uint8_t>& buffer, uint16_t value) {
    buffer.push_back(value >> 8);    // MSB
    buffer.push_back(value & 0xFF);  // LSB
}

void CommandSerializer::writeUint32(std::vector<uint8_t>& buffer, uint32_t value) {
    buffer.push_back(value >> 24);        // MSB
    buffer.push_back((value >> 16) & 0xFF);
    buffer.push_back((value >> 8) & 0xFF);
    buffer.push_back(value & 0xFF);       // LSB
}

uint8_t CommandSerializer::readUint8(const uint8_t* data, size_t& offset) {
    uint8_t value = data[offset];
    offset += 1;
    return value;
}

uint16_t CommandSerializer::readUint16(const uint8_t* data, size_t& offset) {
    uint16_t value = (data[offset] << 8) | data[offset + 1];
    offset += 2;
    return value;
}

uint32_t CommandSerializer::readUint32(const uint8_t* data, size_t& offset) {
    uint32_t value = (data[offset] << 24) | (data[offset + 1] << 16) |
                     (data[offset + 2] << 8) | data[offset + 3];
    offset += 4;
    return value;
}