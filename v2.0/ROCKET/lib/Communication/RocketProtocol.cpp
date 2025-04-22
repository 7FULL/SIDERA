#include "RocketProtocol.h"

// Initialize static members
// Initialize static members
const uint8_t ProtocolPacket::PROTOCOL_VERSION = 0x01;
const uint16_t ProtocolPacket::HEADER_MAGIC = 0xA55A;
uint16_t RocketProtocol::sequenceCounter = 0;

void RocketProtocol::initialize() {
    sequenceCounter = 0;
}

std::vector<uint8_t> RocketProtocol::createCommandPacket(CommandCode code,
                                                         const uint8_t* payload,
                                                         uint16_t length) {
    std::vector<uint8_t> packetData;

    // Header (2 bytes)
    packetData.push_back(ProtocolPacket::HEADER_MAGIC >> 8);    // High byte
    packetData.push_back(ProtocolPacket::HEADER_MAGIC & 0xFF);  // Low byte

    // Version (1 byte)
    packetData.push_back(ProtocolPacket::PROTOCOL_VERSION);

    // Type (1 byte)
    packetData.push_back(static_cast<uint8_t>(code));

    // Sequence number (2 bytes)
    uint16_t sequence = sequenceCounter++;
    packetData.push_back(sequence >> 8);       // High byte
    packetData.push_back(sequence & 0xFF);     // Low byte

    // Length (2 bytes)
    packetData.push_back(length >> 8);         // High byte
    packetData.push_back(length & 0xFF);       // Low byte

    // Payload
    if (payload != nullptr && length > 0) {
        for (uint16_t i = 0; i < length; i++) {
            packetData.push_back(payload[i]);
        }
    }

    // Calculate CRC16 checksum of everything so far
    uint16_t crc = calculateCrc16(packetData.data(), packetData.size());

    // Add checksum (2 bytes)
    packetData.push_back(crc >> 8);            // High byte
    packetData.push_back(crc & 0xFF);          // Low byte

    return packetData;
}

std::vector<uint8_t> RocketProtocol::createResponsePacket(ResponseCode code,
                                                          const uint8_t* payload,
                                                          uint16_t length,
                                                          uint16_t sequenceNumber) {
    std::vector<uint8_t> packetData;

    // Header (2 bytes)
    packetData.push_back(ProtocolPacket::HEADER_MAGIC >> 8);    // High byte
    packetData.push_back(ProtocolPacket::HEADER_MAGIC & 0xFF);  // Low byte

    // Version (1 byte)
    packetData.push_back(ProtocolPacket::PROTOCOL_VERSION);

    // Type (1 byte)
    packetData.push_back(static_cast<uint8_t>(code));

    // Sequence number (2 bytes) - use provided sequence for responses
    packetData.push_back(sequenceNumber >> 8);       // High byte
    packetData.push_back(sequenceNumber & 0xFF);     // Low byte

    // Length (2 bytes)
    packetData.push_back(length >> 8);         // High byte
    packetData.push_back(length & 0xFF);       // Low byte

    // Payload
    if (payload != nullptr && length > 0) {
        for (uint16_t i = 0; i < length; i++) {
            packetData.push_back(payload[i]);
        }
    }

    // Calculate CRC16 checksum of everything so far
    uint16_t crc = calculateCrc16(packetData.data(), packetData.size());

    // Add checksum (2 bytes)
    packetData.push_back(crc >> 8);            // High byte
    packetData.push_back(crc & 0xFF);          // Low byte

    return packetData;
}

bool RocketProtocol::parsePacket(const uint8_t* data, uint16_t length, ProtocolPacket& packet) {
    // Check minimum packet size (header + version + type + sequence + length + checksum = 10 bytes)
    if (length < 10) {
        return false;
    }

    // Check magic header
    uint16_t header = (data[0] << 8) | data[1];
    if (header != ProtocolPacket::HEADER_MAGIC) {
        return false;
    }

    // Parse packet fields
    packet.header = header;
    packet.version = data[2];
    packet.type = data[3];
    packet.sequenceNumber = (data[4] << 8) | data[5];
    packet.length = (data[6] << 8) | data[7];

    // Validate total packet length
    if (length != 8 + packet.length + 2) {  // Header + payload + checksum
        return false;
    }

    // Get payload
    if (packet.length > 0) {
        packet.payload = new uint8_t[packet.length];
        memcpy(packet.payload, &data[8], packet.length);
    } else {
        packet.payload = nullptr;
    }

    // Get checksum from packet
    uint16_t receivedChecksum = (data[8 + packet.length] << 8) | data[8 + packet.length + 1];
    packet.checksum = receivedChecksum;

    // Calculate and verify checksum
    uint16_t calculatedChecksum = calculateCrc16(data, 8 + packet.length);

    return (calculatedChecksum == receivedChecksum);
}

uint16_t RocketProtocol::calculateCrc16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // Initial value

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // XOR with polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool RocketProtocol::verifyPacket(const uint8_t* data, uint16_t length) {
    // Check minimum packet size
    if (length < 10) {
        return false;
    }

    // Get packet length field
    uint16_t payloadLength = (data[6] << 8) | data[7];

    // Check if total length matches
    if (length != 8 + payloadLength + 2) {
        return false;
    }

    // Get checksum from packet
    uint16_t receivedChecksum = (data[8 + payloadLength] << 8) | data[8 + payloadLength + 1];

    // Calculate checksum
    uint16_t calculatedChecksum = calculateCrc16(data, 8 + payloadLength);

    return (calculatedChecksum == receivedChecksum);
}

uint16_t RocketProtocol::getCurrentSequence() {
    return sequenceCounter;
}