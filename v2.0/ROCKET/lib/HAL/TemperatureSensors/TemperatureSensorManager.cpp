/**
 * Temperature Sensor Manager Implementation
 */

#include "TemperatureSensorManager.h"
#include "Config.h"

TemperatureSensorManager::TemperatureSensorManager() {
}

TemperatureSensorManager::~TemperatureSensorManager() {
    // Note: We don't delete the sensors here because they might be used elsewhere
}

void TemperatureSensorManager::addSensor(TemperatureSensor* sensor) {
    Serial.print("Adding temperature sensor: ");
    Serial.println(sensor->getName());
    sensors.push_back(sensor);
}

bool TemperatureSensorManager::begin() {
    if (sensors.empty()) {
        Serial.println("No temperature sensors added to manager");
        return false;
    }

    Serial.println("Starting initialization of temperature sensors...");
    Serial.print("Number of sensors: ");
    Serial.println(sensors.size());

    bool anyInitialized = false;
    int sensorIndex = 1;

    for (auto sensor : sensors) {
        Serial.print("Initializing temperature sensor ");
        Serial.print(sensorIndex++);
        Serial.print(" (");
        Serial.print(sensor->getName());
        Serial.println(")...");

        SensorStatus result = sensor->begin();
        if (result == SensorStatus::OK) {
            Serial.println("Temperature sensor initialization succeeded");
            anyInitialized = true;
        } else {
            Serial.print("[ERROR] Temperature sensor initialization failed with status: ");
            Serial.println(static_cast<int>(result));
        }
    }

    initialized = anyInitialized;
    Serial.print("Temperature sensor initialization complete. Success: ");
    Serial.println(anyInitialized ? "YES" : "NO");
    return anyInitialized;
}

void TemperatureSensorManager::update() {
    if (!initialized) {
        return;
    }

    for (auto sensor : sensors) {
        sensor->update();
    }
}

float TemperatureSensorManager::getTemperature() {
    if (!initialized) {
        return 0.0f;
    }

    // If sensor fusion is enabled and we have multiple operational sensors
    if (USE_SENSOR_FUSION && getOperationalSensorCount() > 1) {
        return fuseTemperatureData() + globalTemperatureOffset;
    }

    // Otherwise, use the first operational sensor
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            return sensor->getTemperature() + globalTemperatureOffset;
        }
    }

    // If no operational sensors found
    return 0.0f + globalTemperatureOffset;
}

int TemperatureSensorManager::getOperationalSensorCount() {
    int count = 0;
    for (auto sensor : sensors) {
        if (sensor->isOperational()) {
            count++;
        }
    }
    return count;
}

void TemperatureSensorManager::setGlobalTemperatureOffset(float offset) {
    globalTemperatureOffset = offset;
}

float TemperatureSensorManager::fuseTemperatureData() {
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