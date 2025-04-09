/**
 * Power Management System Implementation
 */

#include "PowerManager.h"
#include "Config.h"

PowerManager::PowerManager(int batteryPin, StorageManager* storageManager)
        : batteryPin(batteryPin),
          storageManager(storageManager),
          batteryVoltage(0.0f),
          batteryCapacity(1000.0f),         // Default 1000mAh
          batteryFullVoltage(4.2f),         // Default LiPo full voltage
          batteryEmptyVoltage(3.0f),        // Default LiPo empty voltage
          energyUsed(0.0f),
          lowVoltageThreshold(3.3f),        // Default low threshold
          criticalVoltageThreshold(3.0f),   // Default critical threshold
          currentState(PowerState::NORMAL),
          stateChangeCallback(nullptr),
          lastUpdateTime(0),
          lastLogTime(0),
          referenceVoltage(3.3f),           // RP2040 reference voltage
          voltageDividerRatio(2.0f)         // Default 2:1 voltage divider
{
}

bool PowerManager::begin() {
    // Configure ADC pin
    pinMode(batteryPin, INPUT);

    // Initialize voltage reading
    update();

    // Log initialization
    if (storageManager) {
        char message[64];
        snprintf(message, sizeof(message), "Power manager initialized. Battery: %.2fV", batteryVoltage);
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }

    lastUpdateTime = millis();
    lastLogTime = lastUpdateTime;

    return true;
}

void PowerManager::update() {
    // Calculate delta time for energy calculations
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0f / 3600.0f; // Convert to hours
    lastUpdateTime = currentTime;

    // Read battery voltage
    int adcReading = analogRead(batteryPin);
    float newVoltage = calculateBatteryVoltage(adcReading);

    // Apply low-pass filter to reduce noise
    const float FILTER_ALPHA = BATTERY_FILTER_ALPHA;
    batteryVoltage = FILTER_ALPHA * newVoltage + (1.0f - FILTER_ALPHA) * batteryVoltage;

    // Check for battery state changes
    PowerState previousState = currentState;

    if (batteryVoltage <= criticalVoltageThreshold) {
        currentState = PowerState::CRITICAL;
    } else if (batteryVoltage <= lowVoltageThreshold) {
        currentState = PowerState::LOW_POWER;
    } else {
        currentState = PowerState::NORMAL;
    }

    // Handle state changes
    if (currentState != previousState) {
        // Log state change
        if (storageManager) {
            char message[64];
            snprintf(message, sizeof(message),
                     "Power state changed: %d -> %d, Battery: %.2fV",
                     static_cast<int>(previousState),
                     static_cast<int>(currentState),
                     batteryVoltage);
            storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM, message);
        }

        // Call state change callback if registered
        if (stateChangeCallback) {
            stateChangeCallback(currentState);
        }
    }

    // Log battery status periodically
    if (currentTime - lastLogTime >= 60000) { // Log every minute
        lastLogTime = currentTime;

        if (storageManager) {
            char message[64];
            snprintf(message, sizeof(message),
                     "Battery status: %.2fV (%d%%)",
                     batteryVoltage,
                     getBatteryPercentage());
            storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
        }
    }
}

float PowerManager::getBatteryVoltage() const {
    return batteryVoltage;
}

int PowerManager::getBatteryPercentage() const {
    // Calculate percentage based on voltage range
    float percentage = (batteryVoltage - batteryEmptyVoltage) /
                       (batteryFullVoltage - batteryEmptyVoltage) * 100.0f;

    // Clamp to 0-100 range
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    return static_cast<int>(percentage);
}

PowerState PowerManager::getPowerState() const {
    return currentState;
}

bool PowerManager::isBatteryLow() const {
    return currentState == PowerState::LOW_POWER || currentState == PowerState::CRITICAL;
}

bool PowerManager::isBatteryCritical() const {
    return currentState == PowerState::CRITICAL;
}

void PowerManager::enterPowerState(PowerState state) {
    if (state == currentState) {
        return; // Already in this state
    }

    PowerState previousState = currentState;
    currentState = state;

    // Log state change
    if (storageManager) {
        char message[64];
        snprintf(message, sizeof(message),
                 "Power state manually changed: %d -> %d",
                 static_cast<int>(previousState),
                 static_cast<int>(currentState));
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }

    // Call state change callback if registered
    if (stateChangeCallback) {
        stateChangeCallback(currentState);
    }
}

void PowerManager::setLowVoltageThreshold(float volts) {
    lowVoltageThreshold = volts;
}

void PowerManager::setCriticalVoltageThreshold(float volts) {
    criticalVoltageThreshold = volts;
}

void PowerManager::setBatteryCapacity(float mAh) {
    batteryCapacity = mAh;
}

void PowerManager::setBatteryFullVoltage(float volts) {
    batteryFullVoltage = volts;
}

void PowerManager::setBatteryEmptyVoltage(float volts) {
    batteryEmptyVoltage = volts;
}

void PowerManager::setStateChangeCallback(PowerStateCallback callback) {
    stateChangeCallback = callback;
}

float PowerManager::calculateBatteryVoltage(int adcReading) {
    // Convert ADC reading to voltage
    float voltage = (adcReading * referenceVoltage / ADC_RESOLUTION) * voltageDividerRatio;
    return voltage;
}