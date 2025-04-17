/**
 * Flash Storage Implementation
 *
 * Uses W25Q128 SPI Flash memory for storage
 */

#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include "../StorageSystem.h"
#include "StorageConfig.h"

class FlashStorage : public StorageSystem {
public:
    FlashStorage(SPIClass& spi, int8_t csPin);
    ~FlashStorage() override;

    // Implement Sensor interface
    SensorStatus begin() override;
    SensorStatus update() override;
    bool isOperational() override;
    SensorStatus getStatus() const override;
    const char* getName() const override;
    unsigned long getLastReadingTime() const override;

    // Implement StorageSystem interface
    bool logMessage(LogLevel level, Subsystem subsystem, const char* message) override;
    bool storeTelemetry(const StoredTelemetry& telemetry) override;
    bool flush() override;
    uint32_t getTotalCapacity() override;
    uint32_t getAvailableSpace() override;
    const char* getCurrentLogIdentifier() override;
    bool exportDataTo(Stream& outputStream) override;
    bool clearAllData() override;

    // Flash-specific methods
    bool eraseBlock(uint32_t address);

private:
    SPIClass& spi;
    int8_t csPin;
    Adafruit_SPIFlash flash;

    // Memory organization
    static const uint32_t SECTOR_SIZE = FLASH_SECTOR_SIZE;      // Flash sector size in bytes
    static const uint32_t LOG_START_ADDRESS = FLASH_LOG_START_ADDRESS;   // Start address for log data
    static const uint32_t LOG_SIZE = FLASH_LOG_SIZE;      // 64KB for log messages
    static const uint32_t TELEMETRY_START_ADDRESS = FLASH_TELEMETRY_START_ADDRESS; // Start address for telemetry
    static const uint32_t TELEMETRY_SIZE = FLASH_TELEMETRY_SIZE; // 448KB for telemetry
    static const uint32_t METADATA_ADDRESS = LOG_SIZE + TELEMETRY_SIZE; // Metadata address

    // Current indices
    uint32_t currentLogIndex = 0;
    uint32_t currentTelemetryIndex = 0;

    // Buffer for writes
    static const uint16_t BUFFER_SIZE = 256;
    uint8_t writeBuffer[BUFFER_SIZE];
    uint16_t bufferIndex = 0;

    // Flight session info
    uint32_t flightStartTime = 0;
    uint32_t flightEndTime = 0;
    char flightIdentifier[16];

    // Internal methods
    bool initFlash();
    bool readMetadata();
    bool writeMetadata();
    bool flushBuffer();
    uint32_t getTelemetryAddress(uint32_t index);
    uint32_t getLogAddress(uint32_t index);
    bool writeToFlash(uint32_t address, const void* data, uint16_t length);
    bool readFromFlash(uint32_t address, void* data, uint16_t length);
};

#endif // FLASH_STORAGE_H