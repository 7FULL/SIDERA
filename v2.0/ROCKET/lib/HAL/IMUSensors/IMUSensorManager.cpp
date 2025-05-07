/**
 * IMU Sensor Manager Implementation
 */

#include <algorithm>
#include "IMUSensorManager.h"

IMUSensorManager::IMUSensorManager() {
}

IMUSensorManager::~IMUSensorManager() {
    // Note: We don't delete the sensors here because they might be used elsewhere
}

void IMUSensorManager::addSensor(IMUSensor* sensor, uint8_t priority) {
    Serial.print("Adding IMU sensor: ");
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

bool IMUSensorManager::begin() {
    if (sensors.empty()) {
        Serial.println("No IMU sensors added to manager");
        return false;
    }

    Serial.println("Starting initialization of IMU sensors...");
    Serial.print("Number of sensors: ");
    Serial.println(sensors.size());

    bool anyInitialized = false;
    int sensorIndex = 1;

    for (auto& sensorInfo : sensors) {
        Serial.print("Initializing IMU sensor ");
        Serial.print(sensorIndex++);
        Serial.print(" (");
        Serial.print(sensorInfo.sensor->getName());
        Serial.println(")...");

        SensorStatus result = sensorInfo.sensor->begin();
        if (result == SensorStatus::OK) {
            Serial.println("IMU sensor initialization succeeded");
            anyInitialized = true;
        } else {
            Serial.print("[ERROR] IMU sensor initialization failed with status: ");
            Serial.println(static_cast<int>(result));
        }
    }

    initialized = anyInitialized;

    // Find the best sensors to start with
    if (initialized) {
        updateActiveSensors();
        Serial.print("Active accelerometer sensor: ");
        Serial.println(getActiveSensorName());
        Serial.print("Active gyroscope sensor: ");
        Serial.println(getActiveGyroscopeSensorName());
    }

    Serial.print("IMU sensor initialization complete. Success: ");
    Serial.println(anyInitialized ? "YES" : "NO");
    return anyInitialized;
}

void IMUSensorManager::update() {
    if (!initialized) {
        return;
    }

    // Update all sensors
    for (auto& sensorInfo : sensors) {
        sensorInfo.sensor->update();
    }

    // Check if we need to switch active sensors
    updateActiveSensors();
}

bool IMUSensorManager::calibrate() {
    if (!initialized) {
        return false;
    }

    bool allCalibrated = true;

    Serial.println("Calibrating IMU sensors...");

    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            if (sensorInfo.sensor->calibrate() != SensorStatus::OK) {
                allCalibrated = false;
            }
        }
    }

    // Refresh active sensors after calibration
    updateActiveSensors();

    return allCalibrated;
}

AccelerometerData IMUSensorManager::getAccelerometerData() {
    if (!initialized) {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }

    // Use the active accelerometer sensor if available
    if (activeAccelSensor) {
        return activeAccelSensor->getAccelerometerData();
    }

    // Fallback to the first operational sensor
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            return sensorInfo.sensor->getAccelerometerData();
        }
    }

    // If no operational sensors found
    return {0.0f, 0.0f, 0.0f, 0.0f};
}

GyroscopeData IMUSensorManager::getGyroscopeData() {
    if (!initialized || !hasGyroscope()) {
        return {0.0f, 0.0f, 0.0f};
    }

    // Use the active gyroscope sensor if available
    if (activeGyroSensor) {
        return activeGyroSensor->getGyroscopeData();
    }

    // Fallback to the first operational sensor with a gyroscope
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational() && sensorInfo.sensor->hasGyroscope()) {
            return sensorInfo.sensor->getGyroscopeData();
        }
    }

    // If no operational gyroscopes found
    return {0.0f, 0.0f, 0.0f};
}

bool IMUSensorManager::hasGyroscope() const {
    if (!initialized) {
        return false;
    }

    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational() && sensorInfo.sensor->hasGyroscope()) {
            return true;
        }
    }

    return false;
}

int IMUSensorManager::getOperationalSensorCount() {
    int count = 0;
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational()) {
            count++;
        }
    }
    return count;
}

int IMUSensorManager::getOperationalGyroscopeCount() {
    int count = 0;
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational() && sensorInfo.sensor->hasGyroscope()) {
            count++;
        }
    }
    return count;
}

bool IMUSensorManager::detectHighGEvent(float threshold) {
    if (!initialized) {
        return false;
    }

    // Check each sensor for high-G events, prioritizing the active sensor
    if (activeAccelSensor) {
        AccelerometerData data = activeAccelSensor->getAccelerometerData();
        if (data.magnitude > threshold) {
            return true;
        }
    }

    // Check other sensors as backup
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor != activeAccelSensor && sensorInfo.sensor->isOperational()) {
            AccelerometerData data = sensorInfo.sensor->getAccelerometerData();
            if (data.magnitude > threshold) {
                return true;
            }
        }
    }

    return false;
}

const char* IMUSensorManager::getActiveSensorName() {
    if (!initialized || !activeAccelSensor) {
        return "None";
    }

    return activeAccelSensor->getName();
}

const char* IMUSensorManager::getActiveGyroscopeSensorName() {
    if (!initialized || !activeGyroSensor) {
        return "None";
    }

    return activeGyroSensor->getName();
}

void IMUSensorManager::updateActiveSensors() {
    IMUSensor* bestAccelSensor = findBestAccelSensor();
    IMUSensor* bestGyroSensor = findBestGyroSensor();

    // Only switch if there's no active sensor or the best sensor has changed
    if (!activeAccelSensor || (bestAccelSensor && bestAccelSensor != activeAccelSensor)) {
        // Log the sensor change
        if (activeAccelSensor && bestAccelSensor) {
            Serial.print("Switching active accelerometer from ");
            Serial.print(activeAccelSensor->getName());
            Serial.print(" to ");
            Serial.println(bestAccelSensor->getName());
        }

        activeAccelSensor = bestAccelSensor;
    }

    // Same for gyroscope
    if (!activeGyroSensor || (bestGyroSensor && bestGyroSensor != activeGyroSensor)) {
        // Log the sensor change
        if (activeGyroSensor && bestGyroSensor) {
            Serial.print("Switching active gyroscope from ");
            Serial.print(activeGyroSensor->getName());
            Serial.print(" to ");
            Serial.println(bestGyroSensor->getName());
        }

        activeGyroSensor = bestGyroSensor;
    }
}

IMUSensor* IMUSensorManager::findBestAccelSensor() {
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
    return sensors[0].sensor;
}

IMUSensor* IMUSensorManager::findBestGyroSensor() {
    if (sensors.empty()) {
        return nullptr;
    }

    // Return the highest priority sensor that is operational and has a gyroscope
    for (auto& sensorInfo : sensors) {
        if (sensorInfo.sensor->isOperational() && sensorInfo.sensor->hasGyroscope()) {
            return sensorInfo.sensor;
        }
    }

    // If no operational gyroscope sensor, return nullptr
    return nullptr;
}