/**
 * Barometric Sensor Manager Implementation
 */

#include "BarometricSensorManager.h"
#include "Config.h"

BarometricSensorManager::BarometricSensorManager() {
}

BarometricSensorManager::~BarometricSensorManager() {
    // Note: We don't delete the sensors here because they might be used elsewhere
}

void BarometricSensorManager::addSensor(BarometricSensor* sensor) {
    sensors.push_back(sensor);
}

bool BarometricSensorManager::begin() {
    if (sensors.empty()) {
        return false;
    }

    bool anyInitialized = false;

    for (auto sensor : sensors) {
        if (sensor->begin() == SensorStatus::OK) {
            anyInitialized = true;
        }else{
            // Log the error
            Serial.println("Sensor initialization failed");
        }
    }

    initialized = anyInitialized;
    return anyInitialized;
}

void BarometricSensorManager::update() {
    if (!initialized) {
        return;
    }

    for (auto sensor : sensors) {
        sensor->update();
    }
}

void BarometricSensorManager::setReferenceAltitude(float altitude) {
    for (auto sensor : sensors) {
        sensor->setReferenceAltitude(altitude);
    }
}

float BarometricSensorManager::getAltitude() {
    if (!initialized) {
        return 0.0f;
    }

    // If sensor fusion is enabled and we have multiple operational sensors
    if (USE_SENSOR_FUSION && getOperationalSensorCount() > 1) {
        return fuseAltitudeData();
    }

    // Otherwise, use the first operational sensor
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            return sensor->getAltitude();
        }
    }

    // If no operational sensors found
    return 0.0f;
}

float BarometricSensorManager::getPressure() {
    if (!initialized) {
        return 0.0f;
    }

    // If sensor fusion is enabled and we have multiple operational sensors
    if (USE_SENSOR_FUSION && getOperationalSensorCount() > 1) {
        return fusePressureData();
    }

    // Otherwise, use the first operational sensor
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            return sensor->getPressure();
        }
    }

    // If no operational sensors found
    return 0.0f;
}

float BarometricSensorManager::getTemperature() {
    if (!initialized) {
        return 0.0f;
    }

    // If sensor fusion is enabled and we have multiple operational sensors
    if (USE_SENSOR_FUSION && getOperationalSensorCount() > 1) {
        return fuseTemperatureData();
    }

    // Otherwise, use the first operational sensor
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            return sensor->getTemperature();
        }
    }

    // If no operational sensors found
    return 0.0f;
}

int BarometricSensorManager::getOperationalSensorCount() {
    int count = 0;
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            count++;
        }
    }
    return count;
}

float BarometricSensorManager::fuseAltitudeData() {
    float total = 0.0f;
    int count = 0;

    // Simple averaging fusion for now
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            total += sensor->getAltitude();
            count++;
        }
    }

    return (count > 0) ? (total / count) : 0.0f;
}

float BarometricSensorManager::fusePressureData() {
    float total = 0.0f;
    int count = 0;

    // Simple averaging fusion for now
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            total += sensor->getPressure();
            count++;
        }
    }

    return (count > 0) ? (total / count) : 0.0f;
}

float BarometricSensorManager::fuseTemperatureData() {
    float total = 0.0f;
    int count = 0;

    // Simple averaging fusion for now
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            total += sensor->getTemperature();
            count++;
        }
    }

    return (count > 0) ? (total / count) : 0.0f;
}