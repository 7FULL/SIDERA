/**
 * Barometric Sensor Manager Implementation
 */

#include <algorithm>
#include "BarometricSensorManager.h"

BarometricSensorManager::BarometricSensorManager() {
}

BarometricSensorManager::~BarometricSensorManager() {
    // Note: We don't delete the sensors here because they might be used elsewhere
}

void BarometricSensorManager::addSensor(BarometricSensor* sensor, uint8_t priority) {
    Serial.print("Adding barometric sensor: ");
    Serial.print(sensor->getName());
    Serial.print(" with priority ");
    Serial.println(priority);

    SensorInfo info = {sensor, priority};
    sensors.push_back(info);

    // Sort sensors by priority (lower number = higher priority)
    std::sort(sensors.begin(), sensors.end(),
              [](const SensorInfo& a, const SensorInfo& b) {
                  return a.priority < b.priority;
              });
}

bool BarometricSensorManager::begin() {
    if (sensors.empty()) {
        Serial.println("No barometric sensors added to manager");
        return false;
    }

    Serial.println("Starting initialization of barometric sensors...");
    Serial.print("Number of sensors: ");
    Serial.println(sensors.size());

    bool anyInitialized = false;
    int sensorIndex = 1;

    for (auto& sensorInfo : sensors) {
        Serial.print("Initializing sensor ");
        Serial.print(sensorIndex++);
        Serial.print(" (");
        Serial.print(sensorInfo.sensor->getName());
        Serial.println(")...");

        SensorStatus result = sensorInfo.sensor->begin();
        if (result == SensorStatus::OK) {
            Serial.println("Sensor initialization succeeded");
            anyInitialized = true;
        } else {
            Serial.print("[ERROR] Sensor initialization failed with status: ");
            Serial.println(static_cast<int>(result));
        }
    }

    initialized = anyInitialized;

    // Find the best sensor to start with
    if (initialized) {
        updateActiveSensor();
        Serial.print("Active barometric sensor: ");
        Serial.println(getActiveSensorName());
    }

    Serial.print("Barometric sensor initialization complete. Success: ");
    Serial.println(anyInitialized ? "YES" : "NO");
    return anyInitialized;
}

void BarometricSensorManager::update() {
    if (!initialized) {
        return;
    }

    // Update all sensors
    for (auto& sensorInfo : sensors) {
        sensorInfo.sensor->update();
    }

    // Check if we need to switch active sensors
    updateActiveSensor();
}

void BarometricSensorManager::setReferenceAltitude(float altitude) {
    for (auto& sensorInfo : sensors) {
        sensorInfo.sensor->setReferenceAltitude(altitude);
    }
}

float BarometricSensorManager::getAltitude() {
    if (!initialized) {
        return 0.0f;
    }

    // Use the active sensor if available
    if (activeSensor) {
        return activeSensor->getAltitude();
    }

    // Fallback to the first operational sensor
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            return sensorInfo.sensor->getAltitude();
        }
    }

    // If no operational sensors found
    return 0.0f;
}

float BarometricSensorManager::getPressure() {
    if (!initialized) {
        return 0.0f;
    }

    // Use the active sensor if available
    if (activeSensor) {
        return activeSensor->getPressure();
    }

    // Fallback to the first operational sensor
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            return sensorInfo.sensor->getPressure();
        }
    }

    // If no operational sensors found
    return 0.0f;
}

float BarometricSensorManager::getTemperature() {
    if (!initialized) {
        return 0.0f;
    }

    // Use the active sensor if available
    if (activeSensor) {
        return activeSensor->getTemperature();
    }

    // Fallback to the first operational sensor
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            return sensorInfo.sensor->getTemperature();
        }
    }

    // If no operational sensors found
    return 0.0f;
}

int BarometricSensorManager::getOperationalSensorCount() {
    int count = 0;
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            count++;
        }
    }
    return count;
}

const char* BarometricSensorManager::getActiveSensorName() {
    if (!initialized || !activeSensor) {
        return "None";
    }

    return activeSensor->getName();
}

void BarometricSensorManager::updateActiveSensor() {
    BarometricSensor* bestSensor = findBestSensor();

    // Only switch if there's no active sensor or the best sensor has changed
    if (!activeSensor || (bestSensor && bestSensor != activeSensor)) {
        // Log the sensor change
        if (activeSensor && bestSensor) {
            Serial.print("Switching active barometric sensor from ");
            Serial.print(activeSensor->getName());
            Serial.print(" to ");
            Serial.println(bestSensor->getName());
        }

        activeSensor = bestSensor;
    }
}

BarometricSensor* BarometricSensorManager::findBestSensor() {
    if (sensors.empty()) {
        return nullptr;
    }

    // Return the highest priority sensor that is operational
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            return sensorInfo.sensor;
        }
    }

    // If no operational sensor, return the highest priority one anyway
    // (this will be used if all sensors fail)
    return sensors[0].sensor;
}