/**
 * Storage Manager Implementation
 */

#include "StorageManager.h"
#include "SDStorage.h"

StorageManager::StorageManager()
        : primaryStorage(nullptr), currentState(0) {
}

StorageManager::~StorageManager() {
    // Note: We don't delete storage systems here because they might be used elsewhere
}

void StorageManager::addStorage(StorageSystem* storage, bool isPrimary) {
    if (isPrimary) {
        primaryStorage = storage;
    } else {
        secondaryStorage.push_back(storage);
    }
}

bool StorageManager::begin() {
    bool success = true;

    // Initialize primary storage
    if (primaryStorage) {
        success &= (primaryStorage->begin() == SensorStatus::OK);
    }

    // Initialize secondary storage (SD card, etc.)
    for (auto storage : secondaryStorage) {
        // It's okay if secondary storage fails - we'll use primary
        storage->begin();
    }

    return success;
}

bool StorageManager::logMessage(LogLevel level, Subsystem subsystem, const char* message) {
    bool success = true;

    // Always log to primary storage
    if (primaryStorage) {
        success = primaryStorage->logMessage(level, subsystem, message);
    }

    // Log to operational secondary storage systems
    for (auto storage : secondaryStorage) {
        if (storage->isOperational()) {
            storage->logMessage(level, subsystem, message);
        }
    }

    return success;
}

bool StorageManager::storeTelemetry(const StoredTelemetry& telemetry) {
    bool success = true;

    // Always store in primary storage
    if (primaryStorage) {
        success = primaryStorage->storeTelemetry(telemetry);
    }

    // Store in operational secondary storage systems
    for (auto storage : secondaryStorage) {
        if (storage->isOperational()) {
            storage->storeTelemetry(telemetry);
        }
    }

    return success;
}

bool StorageManager::flush() {
    bool success = true;

    // Flush primary storage
    if (primaryStorage) {
        success &= primaryStorage->flush();
    }

    // Flush operational secondary storage systems
    for (auto storage : secondaryStorage) {
        if (storage->isOperational()) {
            storage->flush();
        }
    }

    return success;
}

bool StorageManager::transferData() {
    // Only attempt to transfer if we have a primary storage and at least one secondary
    if (!primaryStorage || secondaryStorage.empty()) {
        return false;
    }

    bool anySuccess = false;

    // Try to transfer to each operational secondary storage
    for (auto storage : secondaryStorage) {
        if (storage->isOperational()) {
            // This assumes SDStorage has an importFromFlash method
            // You might need to add a dynamic_cast here
            if (strcmp(storage->getName(), "SD Card") == 0) {
                anySuccess |= reinterpret_cast<SDStorage*>(storage)->importFromFlash(primaryStorage);
            }
        }
    }

    return anySuccess;
}

void StorageManager::setSystemState(uint8_t state) {
    currentState = state;

    // If we've landed, try to transfer data
    if (state == 7) { // LANDED state value (from your States enum)
        transferData();
    }
}

uint32_t StorageManager::getAvailableSpace() {
    if (primaryStorage) {
        return primaryStorage->getAvailableSpace();
    }
    return 0;
}

bool StorageManager::isOperational() {
    // We consider the system operational if the primary storage is working
    if (primaryStorage) {
        return primaryStorage->isOperational();
    }
    return false;
}

bool StorageManager::clearAllData() {
    bool success = true;

    // Clear primary storage
    if (primaryStorage) {
        success &= primaryStorage->clearAllData();
    }

    // Clear operational secondary storage systems
    for (auto storage : secondaryStorage) {
        if (storage->isOperational()) {
            storage->clearAllData();
        }
    }

    return success;
}

int32_t StorageManager::floatToScaledInt(float value, float scale) {
    return static_cast<int32_t>(value * scale);
}

StoredTelemetry StorageManager::createTelemetryEntry(
        uint8_t state,
        float altitude,
        float vertSpeed,
        float accelX, float accelY, float accelZ,
        float gyroX, float gyroY, float gyroZ,
        float temperature,
        float pressure,
        float batteryVoltage,
        float gpsLat, float gpsLon, float gpsAlt,
        uint8_t gpsSats,
        uint8_t flags
) {
    StoredTelemetry entry;

    entry.timestamp = millis();
    entry.state = state;
    entry.altitude = floatToScaledInt(altitude, 1000.0f);     // Store as mm
    entry.vertSpeed = floatToScaledInt(vertSpeed, 100.0f);    // Store as cm/s
    entry.accelX = floatToScaledInt(accelX, 100.0f);          // Store as 0.01g
    entry.accelY = floatToScaledInt(accelY, 100.0f);
    entry.accelZ = floatToScaledInt(accelZ, 100.0f);
    entry.gyroX = floatToScaledInt(gyroX, 10.0f);             // Store as 0.1 deg/s
    entry.gyroY = floatToScaledInt(gyroY, 10.0f);
    entry.gyroZ = floatToScaledInt(gyroZ, 10.0f);
    entry.temperature = floatToScaledInt(temperature, 10.0f); // Store as 0.1°C
    entry.pressure = floatToScaledInt(pressure, 10.0f);       // Store as 0.1 hPa
    entry.batteryMv = floatToScaledInt(batteryVoltage, 1000.0f); // Store as mV
    entry.gpsLat = floatToScaledInt(gpsLat, 10000000.0f);     // Store as 10^-7 degrees
    entry.gpsLon = floatToScaledInt(gpsLon, 10000000.0f);
    entry.gpsAlt = static_cast<uint16_t>(gpsAlt);             // Store as m
    entry.gpsSats = gpsSats;
    entry.flags = flags;

    return entry;
}