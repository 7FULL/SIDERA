/**
 * SD Card Storage Implementation
 */

#include "SDStorage.h"
#include <cstring>

SDStorage::SDStorage(SPIClass& spi, int8_t csPin)
        : spi(spi), csPin(csPin) {

    // Initialize flight identifier with a timestamp
    snprintf(flightIdentifier, sizeof(flightIdentifier), "F%lu", millis());
}

SDStorage::~SDStorage() {
    // Close any open files
    closeFiles();
}

SensorStatus SDStorage::begin() {
    // Initialize the SD card
    if (!sd.begin(csPin, SD_SCK_MHZ(25))) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    // Create directories if they don't exist
    createDirectory("/logs");
    createDirectory("/telemetry");

    // Initialize a new log session
    if (!initializeSession()) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    status = SensorStatus::OK;
    lastReadingTime = millis();
    return status;
}

SensorStatus SDStorage::update() {
    // Check if SD card is still accessible
    SdSpiConfig config(csPin, DEDICATED_SPI, SD_SCK_MHZ(25), &spi);

    if (!sd.card()->begin(config)) {
        status = SensorStatus::COMMUNICATION_ERROR;
    } else {
        status = SensorStatus::OK;
    }

    lastReadingTime = millis();
    return status;
}

bool SDStorage::isOperational() {
    SdSpiConfig config(csPin, DEDICATED_SPI, SD_SCK_MHZ(25), &spi);
    return status == SensorStatus::OK && sd.card()->begin(config);
}

SensorStatus SDStorage::getStatus() const {
    return status;
}

const char* SDStorage::getName() const {
    return "SD Card";
}

unsigned long SDStorage::getLastReadingTime() const {
    return lastReadingTime;
}

bool SDStorage::logMessage(LogLevel level, Subsystem subsystem, const char* message) {
    if (status != SensorStatus::OK || !logFile.isOpen()) {
        return false;
    }

    // Format: timestamp,level,subsystem,"message"
    char buffer[256]; // Increased buffer size to handle longer messages
    int len = snprintf(buffer, sizeof(buffer), "%lu,%d,%d,\"%s\"\n",
                       millis(), static_cast<int>(level), static_cast<int>(subsystem), message);

    // Check for buffer overflow
    if (len >= sizeof(buffer)) {
        // Message was too long and got truncated
        status = SensorStatus::READING_ERROR;
        return false;
    }

    // Write to file
    if (logFile.write(buffer, len) != len) {
        status = SensorStatus::COMMUNICATION_ERROR;
        return false;
    }

    logCount++;

    // Flush every 10 log messages or if this is a critical/error message
    //TODO
    if (logCount % 10 == 0 || level == LogLevel::CRITICAL || level == LogLevel::ERROR) {
        logFile.flush();
//        if (!logFile.flush()) {
//            status = SensorStatus::COMMUNICATION_ERROR;
//            return false;
//        }
    }

    return true;
}

bool SDStorage::storeTelemetry(const StoredTelemetry& telemetry) {
    if (status != SensorStatus::OK || !telemetryFile.isOpen()) {
        return false;
    }

    // Write binary telemetry data
    size_t bytesWritten = telemetryFile.write(&telemetry, sizeof(StoredTelemetry));
    if (bytesWritten != sizeof(StoredTelemetry)) {
        status = SensorStatus::COMMUNICATION_ERROR;
        return false;
    }

    telemetryCount++;

    // Flush every 50 telemetry entries or if this is a critical state change
    // (like reaching apogee or landing)
    bool isCriticalState = (telemetry.state == static_cast<uint8_t>(RocketState::APOGEE) ||
                            telemetry.state == static_cast<uint8_t>(RocketState::LANDED) ||
                            telemetry.state == static_cast<uint8_t>(RocketState::PARACHUTE_DESCENT));

    if (telemetryCount % 50 == 0 || isCriticalState) {
        telemetryFile.flush();
//        if (!telemetryFile.flush()) {
//            status = SensorStatus::COMMUNICATION_ERROR;
//            return false;
//        }
    }

    return true;
}

bool SDStorage::flush() {
    if (status != SensorStatus::OK) {
        return false;
    }

    bool success = true;

    // Flush log file if open
    if (logFile.isOpen()) {
//        success &= logFile.flush();
        logFile.flush();

        // Set error status if flush failed
        if (!success) {
            status = SensorStatus::COMMUNICATION_ERROR;
            // We can't log the error here as that would create a circular dependency
        }
    }

    // Flush telemetry file if open
    if (telemetryFile.isOpen()) {
//        success &= telemetryFile.flush();
        telemetryFile.flush();

        // Set error status if flush failed
        if (!success) {
            status = SensorStatus::COMMUNICATION_ERROR;
            // We can't log the error here as that would create a circular dependency
        }
    }

    // If flush was successful, reset counters to prevent too frequent flush operations
    if (success) {
        // Reset counters but add a small offset to prevent immediate flush
        // after the next few log entries
        logCount = logCount % 5;
        telemetryCount = telemetryCount % 10;
    }

    return success;
}

uint32_t SDStorage::getTotalCapacity() {
    return sd.card()->sectorCount() * 512ULL;
}

uint32_t SDStorage::getAvailableSpace() {
    int32_t freeClusterCount = sd.vol()->freeClusterCount();
    if (freeClusterCount < 0) {
        return 0;
    }
    return freeClusterCount * sd.vol()->bytesPerCluster();
}

const char* SDStorage::getCurrentLogIdentifier() {
    return flightIdentifier;
}

bool SDStorage::exportDataTo(Stream& outputStream) {
    if (status != SensorStatus::OK) {
        return false;
    }

    // We'd typically export from flash to SD, not from SD to another stream
    // So this method might not be needed for SD storage
    return true;
}

bool SDStorage::clearAllData() {
    if (status != SensorStatus::OK) {
        return false;
    }

    // Close any open files
    closeFiles();

    // Delete log and telemetry files
    sd.remove(currentLogPath);
    sd.remove(currentTelemetryPath);

    // Initialize a new session
    return initializeSession();
}

bool SDStorage::importFromFlash(StorageSystem* flashStorage) {
    if (status != SensorStatus::OK || !flashStorage) {
        return false;
    }

    // Close any existing files
    closeFiles();

    // Use the same flight identifier as the flash storage
    strncpy(flightIdentifier, flashStorage->getCurrentLogIdentifier(), sizeof(flightIdentifier));

    // Create new file paths with the updated identifier
    snprintf(currentLogPath, sizeof(currentLogPath), "/logs/%s.csv", flightIdentifier);
    snprintf(currentTelemetryPath, sizeof(currentTelemetryPath), "/telemetry/%s.dat", flightIdentifier);

    // Open files for writing
    if (!openLogFiles()) {
        return false;
    }

    // Export log data using File32 which is compatible with Stream
    File32 exportFile = sd.open(currentLogPath, O_WRITE | O_CREAT | O_TRUNC);

    if (exportFile) {
        flashStorage->exportDataTo(exportFile);
        exportFile.close();
    } else {
        return false;
    }

    return true;
}

bool SDStorage::createDirectory(const char* path) {
    // Check if directory already exists
    if (sd.exists(path)) {
        return true;
    }

    // Create the directory
    return sd.mkdir(path);
}

bool SDStorage::listFiles(const char* path, Stream& outputStream) {
    FatFile dir, file;

    if (!dir.open(path)) {
        outputStream.print("Failed to open directory: ");
        outputStream.println(path);
        return false;
    }

    outputStream.print("Directory listing for: ");
    outputStream.println(path);
    outputStream.println("-------------------");

    // Print all files in directory
    dir.rewind();
    while (file.openNext(&dir, O_READ)) {
        char name[32];
        file.getName(name, sizeof(name));

        // Print file name and size
        if (!file.isDir()) {
            outputStream.print(name);
            outputStream.print("\t");
            outputStream.println(file.fileSize());
        } else {
            outputStream.print("[DIR] ");
            outputStream.println(name);
        }

        file.close();
    }

    dir.close();
    return true;
}

bool SDStorage::initializeSession() {
    // Create a new flight identifier
    snprintf(flightIdentifier, sizeof(flightIdentifier), "F%lu", millis());

    // Create file paths
    snprintf(currentLogPath, sizeof(currentLogPath), "/logs/%s.csv", flightIdentifier);
    snprintf(currentTelemetryPath, sizeof(currentTelemetryPath), "/telemetry/%s.dat", flightIdentifier);

    // Open log files
    return openLogFiles();
}

bool SDStorage::openLogFiles() {
    // Close any previously open files
    closeFiles();

    // Open log file with header
    if (!logFile.open(currentLogPath, O_WRITE | O_CREAT | O_TRUNC)) {
        return false;
    }

    // Write log file header
    const char* header = "timestamp,level,subsystem,message\n";
    if (logFile.write(header, strlen(header)) != strlen(header)) {
        logFile.close();
        return false;
    }

    // Open telemetry file
    if (!telemetryFile.open(currentTelemetryPath, O_WRITE | O_CREAT | O_TRUNC)) {
        logFile.close();
        return false;
    }

    return true;
}

void SDStorage::closeFiles() {
    if (logFile.isOpen()) {
        logFile.close();
    }

    if (telemetryFile.isOpen()) {
        telemetryFile.close();
    }
}