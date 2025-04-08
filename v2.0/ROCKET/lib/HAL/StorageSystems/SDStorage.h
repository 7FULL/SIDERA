/**
 * SD Card Storage Implementation
 *
 * Uses SdFat to access the SD card
 */

#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <SD.h>
#include "../StorageSystem.h"

class SDStorage : public StorageSystem {
public:
    SDStorage(SPIClass& spi, int8_t csPin);
    ~SDStorage() override;

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

    // SD-specific methods
    bool importFromFlash(StorageSystem* flashStorage);
    bool createDirectory(const char* path);
    bool listFiles(const char* path, Stream& outputStream);

private:
    SPIClass& spi;
    int8_t csPin;
    SdFat sd;
    File32  logFile;
    File32  telemetryFile;

    char currentLogPath[32];
    char currentTelemetryPath[32];
    char flightIdentifier[16];

    uint32_t logCount = 0;
    uint32_t telemetryCount = 0;

    // Initialize a new log session
    bool initializeSession();

    // Open log files with appropriate names
    bool openLogFiles();

    // Close any open files
    void closeFiles();
};

#endif // SD_STORAGE_H