/**
 * Flash Storage Implementation
 */

#include "FlashStorage.h"
#include <string.h>

FlashStorage::FlashStorage(SPIClass& spi, int8_t csPin)
        : spi(spi), csPin(csPin), flash() {

    // Initialize flight identifier with a timestamp
    snprintf(flightIdentifier, sizeof(flightIdentifier), "F%lu", millis());
}

FlashStorage::~FlashStorage() {
    // Make sure any buffered data is written
    flush();
}

SensorStatus FlashStorage::begin() {
    // Initialize the flash memory
    if (!initFlash()) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    // Read metadata to restore indices
    if (!readMetadata()) {
        // If reading metadata fails, assume this is the first use
        currentLogIndex = 0;
        currentTelemetryIndex = 0;
        flightStartTime = millis();
        flightEndTime = 0;

        // Write initial metadata
        writeMetadata();
    }

    bufferIndex = 0;
    status = SensorStatus::OK;
    lastReadingTime = millis();
    return status;
}

SensorStatus FlashStorage::update() {
    // Nothing to do in update for storage
    lastReadingTime = millis();
    return status;
}

bool FlashStorage::isOperational() {
    return status == SensorStatus::OK;
}

SensorStatus FlashStorage::getStatus() const {
    return status;
}

const char* FlashStorage::getName() const {
    return "W25Q128 Flash";
}

unsigned long FlashStorage::getLastReadingTime() const {
    return lastReadingTime;
}

bool FlashStorage::logMessage(LogLevel level, Subsystem subsystem, const char* message) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Create log entry
    LogEntry entry;
    entry.timestamp = millis();
    entry.logLevel = static_cast<uint8_t>(level);
    entry.subsystem = static_cast<uint8_t>(subsystem);

    // Copy message (truncate if necessary)
    strncpy(entry.message, message, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';

    // Write to flash
    uint32_t address = getLogAddress(currentLogIndex);
    bool success = writeToFlash(address, &entry, sizeof(LogEntry));

    if (success) {
        currentLogIndex++;
        // Update metadata every 10 log entries
        if (currentLogIndex % 10 == 0) {
            writeMetadata();
        }
    }

    return success;
}

bool FlashStorage::storeTelemetry(const StoredTelemetry& telemetry) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Write to flash
    uint32_t address = getTelemetryAddress(currentTelemetryIndex);
    bool success = writeToFlash(address, &telemetry, sizeof(StoredTelemetry));

    if (success) {
        currentTelemetryIndex++;
        // Update metadata every 50 telemetry entries
        if (currentTelemetryIndex % 50 == 0) {
            writeMetadata();
        }
    }

    return success;
}

bool FlashStorage::flush() {
    if (status != SensorStatus::OK) {
        return false;
    }

    bool success = flushBuffer();

    // Update metadata to ensure we know where we left off
    writeMetadata();

    return success;
}

uint32_t FlashStorage::getTotalCapacity() {
    return flash.size();
}

uint32_t FlashStorage::getAvailableSpace() {
    uint32_t usedLogSpace = currentLogIndex * sizeof(LogEntry);
    uint32_t usedTelemetrySpace = currentTelemetryIndex * sizeof(StoredTelemetry);
    uint32_t totalUsed = usedLogSpace + usedTelemetrySpace;

    return flash.size() - totalUsed;
}

const char* FlashStorage::getCurrentLogIdentifier() {
    return flightIdentifier;
}

bool FlashStorage::exportDataTo(Stream& outputStream) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Make sure all data is written to flash
    flush();

    // Write header to the output stream
    outputStream.println("Rocket Flight Data Export");
    outputStream.print("Flight ID: ");
    outputStream.println(flightIdentifier);
    outputStream.print("Start Time: ");
    outputStream.println(flightStartTime);
    outputStream.print("End Time: ");
    outputStream.println(flightEndTime);
    outputStream.println();

    // Export log messages
    outputStream.println("--- LOG MESSAGES ---");
    outputStream.println("Timestamp,Level,Subsystem,Message");

    LogEntry logEntry;
    for (uint32_t i = 0; i < currentLogIndex; i++) {
        uint32_t address = getLogAddress(i);
        if (readFromFlash(address, &logEntry, sizeof(LogEntry))) {
            // Write CSV formatted log entry
            outputStream.print(logEntry.timestamp);
            outputStream.print(",");
            outputStream.print(logEntry.logLevel);
            outputStream.print(",");
            outputStream.print(logEntry.subsystem);
            outputStream.print(",\"");
            outputStream.print(logEntry.message);
            outputStream.println("\"");
        }
    }

    // Export telemetry data
    outputStream.println("\n--- TELEMETRY DATA ---");
    outputStream.println("Timestamp,State,Altitude,VertSpeed,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temp,Pressure,BatteryMv,GpsLat,GpsLon,GpsAlt,GpsSats,Flags");

    StoredTelemetry telemetry;
    for (uint32_t i = 0; i < currentTelemetryIndex; i++) {
        uint32_t address = getTelemetryAddress(i);
        if (readFromFlash(address, &telemetry, sizeof(StoredTelemetry))) {
            // Write CSV formatted telemetry entry
            outputStream.print(telemetry.timestamp);
            outputStream.print(",");
            outputStream.print(telemetry.state);
            outputStream.print(",");
            outputStream.print(telemetry.altitude / 1000.0f); // Convert back to meters
            outputStream.print(",");
            outputStream.print(telemetry.vertSpeed / 100.0f); // Convert back to m/s
            outputStream.print(",");
            outputStream.print(telemetry.accelX / 100.0f); // Convert back to g
            outputStream.print(",");
            outputStream.print(telemetry.accelY / 100.0f);
            outputStream.print(",");
            outputStream.print(telemetry.accelZ / 100.0f);
            outputStream.print(",");
            outputStream.print(telemetry.gyroX / 10.0f); // Convert back to deg/s
            outputStream.print(",");
            outputStream.print(telemetry.gyroY / 10.0f);
            outputStream.print(",");
            outputStream.print(telemetry.gyroZ / 10.0f);
            outputStream.print(",");
            outputStream.print(telemetry.temperature / 10.0f); // Convert back to °C
            outputStream.print(",");
            outputStream.print(telemetry.pressure / 10.0f); // Convert back to hPa
            outputStream.print(",");
            outputStream.print(telemetry.batteryMv / 1000.0f); // Convert to V
            outputStream.print(",");
            outputStream.print(telemetry.gpsLat / 10000000.0f); // Convert back to degrees
            outputStream.print(",");
            outputStream.print(telemetry.gpsLon / 10000000.0f);
            outputStream.print(",");
            outputStream.print(telemetry.gpsAlt);
            outputStream.print(",");
            outputStream.print(telemetry.gpsSats);
            outputStream.print(",");
            outputStream.println(telemetry.flags, HEX);
        }
    }

    // Mark the end of the export
    outputStream.println("\n--- END OF EXPORT ---");

    return true;
}

bool FlashStorage::clearAllData() {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Erase log section
    for (uint32_t addr = LOG_START_ADDRESS; addr < LOG_START_ADDRESS + LOG_SIZE; addr += SECTOR_SIZE) {
        if (!eraseBlock(addr)) {
            return false;
        }
    }

    // Erase telemetry section
    for (uint32_t addr = TELEMETRY_START_ADDRESS; addr < TELEMETRY_START_ADDRESS + TELEMETRY_SIZE; addr += SECTOR_SIZE) {
        if (!eraseBlock(addr)) {
            return false;
        }
    }

    // Reset indices
    currentLogIndex = 0;
    currentTelemetryIndex = 0;
    flightStartTime = millis();
    flightEndTime = 0;

    // Write initial metadata
    return writeMetadata();
}

bool FlashStorage::eraseBlock(uint32_t address) {
    if (!flash.eraseSector(address / SECTOR_SIZE)) {
        status = SensorStatus::COMMUNICATION_ERROR;
        return false;
    }
    return true;
}

bool FlashStorage::initFlash() {
    if (!flash.begin()) {
        return false;
    }

    // Check if the flash chip is recognized
    if (flash.size() == 0) {
        return false;
    }

    return true;
}

bool FlashStorage::readMetadata() {
    // Metadata structure:
    // uint32_t magicNumber
    // uint32_t currentLogIndex
    // uint32_t currentTelemetryIndex
    // uint32_t flightStartTime
    // uint32_t flightEndTime
    // char flightIdentifier[16]

    const uint32_t MAGIC = 0x46534D44; // "FSMD" in hex

    uint8_t buffer[32 + 16]; // Enough space for metadata

    if (!readFromFlash(METADATA_ADDRESS, buffer, sizeof(buffer))) {
        return false;
    }

    uint32_t magic;
    memcpy(&magic, buffer, sizeof(magic));

    if (magic != MAGIC) {
        return false; // Not a valid metadata block
    }

    memcpy(&currentLogIndex, buffer + 4, sizeof(currentLogIndex));
    memcpy(&currentTelemetryIndex, buffer + 8, sizeof(currentTelemetryIndex));
    memcpy(&flightStartTime, buffer + 12, sizeof(flightStartTime));
    memcpy(&flightEndTime, buffer + 16, sizeof(flightEndTime));
    memcpy(flightIdentifier, buffer + 20, 16);

    return true;
}

bool FlashStorage::writeMetadata() {
    // Metadata structure:
    // uint32_t magicNumber
    // uint32_t currentLogIndex
    // uint32_t currentTelemetryIndex
    // uint32_t flightStartTime
    // uint32_t flightEndTime
    // char flightIdentifier[16]

    const uint32_t MAGIC = 0x46534D44; // "FSMD" in hex

    uint8_t buffer[32 + 16]; // Enough space for metadata

    memcpy(buffer, &MAGIC, sizeof(MAGIC));
    memcpy(buffer + 4, &currentLogIndex, sizeof(currentLogIndex));
    memcpy(buffer + 8, &currentTelemetryIndex, sizeof(currentTelemetryIndex));
    memcpy(buffer + 12, &flightStartTime, sizeof(flightStartTime));

    // Update flight end time
    flightEndTime = millis();

    memcpy(buffer + 16, &flightEndTime, sizeof(flightEndTime));
    memcpy(buffer + 20, flightIdentifier, 16);

    // Erase metadata sector first
    if (!eraseBlock(METADATA_ADDRESS)) {
        return false;
    }

    // Write metadata
    return writeToFlash(METADATA_ADDRESS, buffer, sizeof(buffer));
}

bool FlashStorage::flushBuffer() {
    // If there's data in the buffer, write it to flash
    if (bufferIndex > 0) {
        // This is just a placeholder - in a real implementation,
        // you would have a proper buffer and write logic
        bufferIndex = 0;
    }

    return true;
}

uint32_t FlashStorage::getTelemetryAddress(uint32_t index) {
    return TELEMETRY_START_ADDRESS + (index * sizeof(StoredTelemetry));
}

uint32_t FlashStorage::getLogAddress(uint32_t index) {
    return LOG_START_ADDRESS + (index * sizeof(LogEntry));
}

bool FlashStorage::writeToFlash(uint32_t address, const void* data, uint16_t length) {
    // Check if we're about to exceed flash size
    if (address + length > flash.size()) {
        // Storage full - perhaps implement circular buffer behavior here
        return false;
    }

    if (!flash.writeBuffer(address, (const uint8_t*)data, length)) {
        status = SensorStatus::COMMUNICATION_ERROR;
        return false;
    }

    return true;
}

bool FlashStorage::readFromFlash(uint32_t address, void* data, uint16_t length) {
    if (address + length > flash.size()) {
        return false;
    }

    if (!flash.readBuffer(address, (uint8_t*)data, length)) {
        status = SensorStatus::COMMUNICATION_ERROR;
        return false;
    }

    return true;
}