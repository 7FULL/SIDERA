/**
 * Storage Manager
 *
 * Manages multiple storage systems and provides a unified interface
 */

#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include "../StorageSystem.h"
#include <vector>

class StorageManager {
public:
    StorageManager();
    ~StorageManager();

    // Add a storage system (flash, SD card, etc.)
    void addStorage(StorageSystem* storage, bool isPrimary = false);

    // Initialize all storage systems
    bool begin();

    // Log a message to all active storage systems
    bool logMessage(LogLevel level, Subsystem subsystem, const char* message);

    // Store telemetry data in all active storage systems
    bool storeTelemetry(const StoredTelemetry& telemetry);

    // Flush all storage systems
    bool flush();

    // Transfer data from primary to secondary storage (e.g., Flash to SD)
    bool transferData();

    // Set the system state (to allow tracking LANDED state)
    void setSystemState(uint8_t state);

    // Get the available space in the primary storage
    uint32_t getAvailableSpace();

    // Check if all storage systems are operational
    bool isOperational();

    // Clear all data in all storage systems
    bool clearAllData();

private:
    StorageSystem* primaryStorage;
    std::vector<StorageSystem*> secondaryStorage;
    uint8_t currentState;

    // Helper method to convert float values to scaled integers for storage
    static int32_t floatToScaledInt(float value, float scale);

    // Helper method to create a StoredTelemetry struct from raw sensor values
    static StoredTelemetry createTelemetryEntry(
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
    );
};

#endif // STORAGE_MANAGER_H