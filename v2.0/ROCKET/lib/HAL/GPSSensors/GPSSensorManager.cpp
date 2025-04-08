/**
 * GPS Sensor Manager Implementation
 */

#include "GPSSensorManager.h"

GPSSensorManager::GPSSensorManager() {
    // Initialize last valid data
    lastValidData = {0};
    lastValidData.valid = false;
}

GPSSensorManager::~GPSSensorManager() {
    // Note: We don't delete the sensors here because they might be used elsewhere
}

void GPSSensorManager::addSensor(GPSSensor* sensor, uint8_t priority) {
    GPSSensorInfo info = {sensor, priority};
    sensors.push_back(info);

    // Sort sensors by priority (lower number = higher priority)
    std::sort_heap(sensors.begin(), sensors.end(),
              [](const GPSSensorInfo& a, const GPSSensorInfo& b) {
                  return a.priority < b.priority;
              });
}

bool GPSSensorManager::begin() {
    if (sensors.empty()) {
        return false;
    }

    bool anyInitialized = false;

    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->begin() == SensorStatus::OK) {
            anyInitialized = true;
        }
    }

    initialized = anyInitialized;

    // Find the best sensor to start with
    if (initialized) {
        updateActiveSensor();
    }

    return anyInitialized;
}

void GPSSensorManager::update() {
    if (!initialized) {
        return;
    }

    // Update all sensors
    for (auto& sensorInfo : sensors) {
        sensorInfo.sensor->update();
    }

    // Check if we need to switch active sensors
    updateActiveSensor();

    // Get data from active sensor and save it if valid
    if (activeSensor && activeSensor->hasPositionFix()) {
        GPSData currentData = activeSensor->getGPSData();
        if (currentData.valid) {
            lastValidData = currentData;
        }
    }
}

GPSData GPSSensorManager::getGPSData() {
    if (!initialized) {
        return {0};
    }

    if (activeSensor) {
        GPSData currentData = activeSensor->getGPSData();
        if (currentData.valid) {
            return currentData;
        }
    }

    // Return the last valid data if we have it
    return lastValidData;
}

bool GPSSensorManager::hasPositionFix() {
    if (!initialized || !activeSensor) {
        return false;
    }

    return activeSensor->hasPositionFix();
}

unsigned long GPSSensorManager::getFixAge() {
    if (!initialized || !activeSensor) {
        return ULONG_MAX;
    }

    return activeSensor->getFixAge();
}

uint8_t GPSSensorManager::getSatelliteCount() {
    if (!initialized || !activeSensor) {
        return 0;
    }

    return activeSensor->getSatelliteCount();
}

bool GPSSensorManager::enableLowPowerMode() {
    if (!initialized) {
        return false;
    }

    bool success = true;

    for (auto& sensorInfo : sensors) {
        if (!sensorInfo.sensor->enableLowPowerMode()) {
            success = false;
        }
    }

    return success;
}

bool GPSSensorManager::disableLowPowerMode() {
    if (!initialized) {
        return false;
    }

    bool success = true;

    for (auto& sensorInfo : sensors) {
        if (!sensorInfo.sensor->disableLowPowerMode()) {
            success = false;
        }
    }

    // After waking up, find the best sensor
    updateActiveSensor();

    return success;
}

const char* GPSSensorManager::getActiveSensorName() {
    if (!initialized || !activeSensor) {
        return "None";
    }

    return activeSensor->getName();
}

int GPSSensorManager::getOperationalSensorCount() {
    if (!initialized) {
        return 0;
    }

    int count = 0;
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            count++;
        }
    }

    return count;
}

void GPSSensorManager::updateActiveSensor() {
    GPSSensor* bestSensor = findBestSensor();

    // Only switch if there's no active sensor or the best sensor has changed
    if (!activeSensor || (bestSensor && bestSensor != activeSensor)) {
        activeSensor = bestSensor;
    }
}

GPSSensor* GPSSensorManager::findBestSensor() {
    if (sensors.empty()) {
        return nullptr;
    }

    // First, check for sensors with a valid position fix
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational() && sensorInfo.sensor->hasPositionFix()) {
            return sensorInfo.sensor;
        }
    }

    // If no sensor has a fix, use the highest priority operational sensor
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            return sensorInfo.sensor;
        }
    }

    // If no operational sensor, use the highest priority sensor regardless
    if (!sensors.empty()) {
        return sensors[0].sensor;
    }

    return nullptr;
}