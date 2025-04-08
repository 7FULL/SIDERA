/**
 * Telemetry Serializer Implementation
 */

#include "TelemetrySerializer.h"

std::vector<uint8_t> TelemetrySerializer::serialize(const TelemetryPacket& packet) {
    std::vector<uint8_t> buffer;
    buffer.reserve(50); // Approximate size needed

    // Write all fields to the buffer
    writeUint32(buffer, packet.timestamp);
    writeUint8(buffer, packet.rocketState);
    writeFloat(buffer, packet.altitude);
    writeFloat(buffer, packet.verticalSpeed);
    writeFloat(buffer, packet.acceleration);
    writeFloat(buffer, packet.temperature);
    writeFloat(buffer, packet.pressure);
    writeFloat(buffer, packet.batteryVoltage);
    writeUint8(buffer, packet.gpsSatellites);
    writeFloat(buffer, packet.gpsLatitude);
    writeFloat(buffer, packet.gpsLongitude);
    writeFloat(buffer, packet.gpsAltitude);
    writeUint8(buffer, packet.sensorStatus);
    writeUint8(buffer, packet.flags);

    return buffer;
}

TelemetryPacket TelemetrySerializer::deserialize(const uint8_t* data, size_t length) {
    TelemetryPacket packet;
    size_t offset = 0;

    // Read all fields from the buffer
    if (offset + 4 <= length) packet.timestamp = readUint32(data, offset);
    if (offset + 1 <= length) packet.rocketState = readUint8(data, offset);
    if (offset + 4 <= length) packet.altitude = readFloat(data, offset);
    if (offset + 4 <= length) packet.verticalSpeed = readFloat(data, offset);
    if (offset + 4 <= length) packet.acceleration = readFloat(data, offset);
    if (offset + 4 <= length) packet.temperature = readFloat(data, offset);
    if (offset + 4 <= length) packet.pressure = readFloat(data, offset);
    if (offset + 4 <= length) packet.batteryVoltage = readFloat(data, offset);
    if (offset + 1 <= length) packet.gpsSatellites = readUint8(data, offset);
    if (offset + 4 <= length) packet.gpsLatitude = readFloat(data, offset);
    if (offset + 4 <= length) packet.gpsLongitude = readFloat(data, offset);
    if (offset + 4 <= length) packet.gpsAltitude = readFloat(data, offset);
    if (offset + 1 <= length) packet.sensorStatus = readUint8(data, offset);
    if (offset + 1 <= length) packet.flags = readUint8(data, offset);

    return packet;
}

TelemetryPacket TelemetrySerializer::createPacket(
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
) {
    TelemetryPacket packet;

    packet.timestamp = millis();
    packet.rocketState = rocketState;
    packet.altitude = altitude;
    packet.verticalSpeed = verticalSpeed;
    packet.acceleration = acceleration;
    packet.temperature = temperature;
    packet.pressure = pressure;
    packet.batteryVoltage = batteryVoltage;
    packet.gpsSatellites = gpsSatellites;
    packet.gpsLatitude = gpsLatitude;
    packet.gpsLongitude = gpsLongitude;
    packet.gpsAltitude = gpsAltitude;
    packet.sensorStatus = sensorStatus;
    packet.flags = flags;

    return packet;
}

void TelemetrySerializer::writeUint8(std::vector<uint8_t>& buffer, uint8_t value) {
    buffer.push_back(value);
}

void TelemetrySerializer::writeUint16(std::vector<uint8_t>& buffer, uint16_t value) {
    buffer.push_back(value >> 8);    // MSB
    buffer.push_back(value & 0xFF);  // LSB
}

void TelemetrySerializer::writeUint32(std::vector<uint8_t>& buffer, uint32_t value) {
    buffer.push_back(value >> 24);        // MSB
    buffer.push_back((value >> 16) & 0xFF);
    buffer.push_back((value >> 8) & 0xFF);
    buffer.push_back(value & 0xFF);       // LSB
}

void TelemetrySerializer::writeFloat(std::vector<uint8_t>& buffer, float value) {
    // Convert float to byte representation
    uint8_t* bytes = (uint8_t*)&value;

    // Write bytes to buffer
    for (int i = 0; i < 4; i++) {
        buffer.push_back(bytes[i]);
    }
}

uint8_t TelemetrySerializer::readUint8(const uint8_t* data, size_t& offset) {
    uint8_t value = data[offset];
    offset += 1;
    return value;
}

uint16_t TelemetrySerializer::readUint16(const uint8_t* data, size_t& offset) {
    uint16_t value = (data[offset] << 8) | data[offset + 1];
    offset += 2;
    return value;
}

uint32_t TelemetrySerializer::readUint32(const uint8_t* data, size_t& offset) {
    uint32_t value = (data[offset] << 24) | (data[offset + 1] << 16) |
                     (data[offset + 2] << 8) | data[offset + 3];
    offset += 4;
    return value;
}

float TelemetrySerializer::readFloat(const uint8_t* data, size_t& offset) {
    // Create a union to convert bytes to float
    union {
        float f;
        uint8_t bytes[4];
    } converter;

    // Copy bytes from data to the union
    for (int i = 0; i < 4; i++) {
        converter.bytes[i] = data[offset + i];
    }

    offset += 4;
    return converter.f;
}