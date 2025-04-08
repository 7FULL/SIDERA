/**
 * IMU Sensor Manager Implementation
 */

#include "IMUSensorManager.h"
#include "Config.h"

IMUSensorManager::IMUSensorManager() {
}

IMUSensorManager::~IMUSensorManager() {
    // Note: We don't delete the sensors here because they might be used elsewhere
}

void IMUSensorManager::addSensor(IMUSensor* sensor) {
    sensors.push_back(sensor);
}

bool IMUSensorManager::begin() {
    if (sensors.empty()) {
        return false;
    }

    bool anyInitialized = false;

    for (auto sensor : sensors) {
        if (sensor->begin() == SensorStatus::OK) {
            anyInitialized = true;
        }
    }

    initialized = anyInitialized;
    return anyInitialized;
}

void IMUSensorManager::update() {
    if (!initialized) {
        return;
    }

    for (auto sensor : sensors) {
        sensor->update();
    }
}

bool IMUSensorManager::calibrate() {
    if (!initialized) {
        return false;
    }

    bool allCalibrated = true;

    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            if (sensor->calibrate() != SensorStatus::OK) {
                allCalibrated = false;
            }
        }
    }

    return allCalibrated;
}

AccelerometerData IMUSensorManager::getAccelerometerData() {
    if (!initialized) {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }

    // If sensor fusion is enabled and we have multiple operational sensors
    if (USE_SENSOR_FUSION && getOperationalSensorCount() > 1) {
        return fuseAccelerometerData();
    }

    // Otherwise, use the first operational sensor
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            return sensor->getAccelerometerData();
        }
    }

    // If no operational sensors found
    return {0.0f, 0.0f, 0.0f, 0.0f};
}

GyroscopeData IMUSensorManager::getGyroscopeData() {
    if (!initialized || !hasGyroscope()) {
        return {0.0f, 0.0f, 0.0f};
    }

    // If sensor fusion is enabled and we have multiple operational gyroscopes
    if (USE_SENSOR_FUSION && getOperationalGyroscopeCount() > 1) {
        return fuseGyroscopeData();
    }

    // Otherwise, use the first operational sensor with a gyroscope
    for (auto sensor : sensors) {
        if (sensor->isOperational() && sensor->hasGyroscope()) {
            return sensor->getGyroscopeData();
        }
    }

    // If no operational gyroscopes found
    return {0.0f, 0.0f, 0.0f};
}

bool IMUSensorManager::hasGyroscope() const {
    if (!initialized) {
        return false;
    }

    for (auto sensor : sensors) {
        if (sensor->isOperational() && sensor->hasGyroscope()) {
            return true;
        }
    }

    return false;
}

int IMUSensorManager::getOperationalSensorCount() {
    int count = 0;
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            count++;
        }
    }
    return count;
}

int IMUSensorManager::getOperationalGyroscopeCount() {
    int count = 0;
    for (auto sensor : sensors) {
        if (sensor->isOperational() && sensor->hasGyroscope()) {
            count++;
        }
    }
    return count;
}

bool IMUSensorManager::detectHighGEvent(float threshold) {
    if (!initialized) {
        return false;
    }

    // Check each sensor for high-G events
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            AccelerometerData data = sensor->getAccelerometerData();
            if (data.magnitude > threshold) {
                return true;
            }
        }
    }

    return false;
}

AccelerometerData IMUSensorManager::fuseAccelerometerData() {
    AccelerometerData result = {0.0f, 0.0f, 0.0f, 0.0f};
    int count = 0;

    // For now, we use a simple complementary filter that gives preference
    // to high-G sensors when acceleration is high, and to low-G sensors
    // when acceleration is low

    // First pass: collect data from all sensors
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            AccelerometerData data = sensor->getAccelerometerData();

            // Apply weighting based on sensor range
            float weight = 1.0f;

            // If it's a high-G sensor, give it more weight when acceleration is high
            float range = sensor->getAccelerometerRange();
            if (range > 50.0f) { // High-G sensor
                weight = 0.2f + 0.8f * min(1.0f, data.magnitude / 30.0f);
            } else { // Low-G sensor
                weight = 1.0f - 0.8f * min(1.0f, data.magnitude / 30.0f);
            }

            result.x += data.x * weight;
            result.y += data.y * weight;
            result.z += data.z * weight;
            count += weight;
        }
    }

    // Calculate weighted average
    if (count > 0) {
        result.x /= count;
        result.y /= count;
        result.z /= count;

        // Apply a simple low-pass filter to reduce noise
        const float alpha = 0.2f; // Filter constant
        result.x = alpha * result.x + (1.0f - alpha) * lastAccelX;
        result.y = alpha * result.y + (1.0f - alpha) * lastAccelY;
        result.z = alpha * result.z + (1.0f - alpha) * lastAccelZ;

        // Save current values for next filter pass
        lastAccelX = result.x;
        lastAccelY = result.y;
        lastAccelZ = result.z;

        // Calculate magnitude
        result.magnitude = sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
    }

    return result;
}

GyroscopeData IMUSensorManager::fuseGyroscopeData() {
    GyroscopeData result = {0.0f, 0.0f, 0.0f};
    int count = 0;

    // Simple averaging fusion for now
    for (auto sensor : sensors) {
        if (sensor->isOperational() && sensor->hasGyroscope()) {
            GyroscopeData data = sensor->getGyroscopeData();
            result.x += data.x;
            result.y += data.y;
            result.z += data.z;
            count++;
        }
    }

    if (count > 0) {
        result.x /= count;
        result.y /= count;
        result.z /= count;
    }

    return result;
}