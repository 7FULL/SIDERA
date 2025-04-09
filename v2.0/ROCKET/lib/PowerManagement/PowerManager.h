/**
 * Power Management System
 *
 * Monitors battery voltage and manages power modes
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include "../HAL/StorageSystems/StorageManager.h"

// Power states
enum class PowerState {
    NORMAL,       // Normal operation, all systems active
    LOW_POWER,    // Low power mode, non-critical systems reduced
    CRITICAL      // Critical power mode, only essential systems active
};

class PowerManager {
public:
    PowerManager(int batteryPin, StorageManager* storageManager = nullptr);

    // Initialize the power manager
    bool begin();

    // Update battery status
    void update();

    // Get current battery voltage
    float getBatteryVoltage() const;

    // Get battery percentage (0-100%)
    int getBatteryPercentage() const;

    // Get current power state
    PowerState getPowerState() const;

    // Check if battery is low
    bool isBatteryLow() const;

    // Check if battery is critical
    bool isBatteryCritical() const;

    // Enter specific power state
    void enterPowerState(PowerState state);

    // Configure voltage thresholds
    void setLowVoltageThreshold(float volts);
    void setCriticalVoltageThreshold(float volts);

    // Configure battery parameters
    void setBatteryCapacity(float mAh);
    void setBatteryFullVoltage(float volts);
    void setBatteryEmptyVoltage(float volts);

    // Register callback for power state changes
    using PowerStateCallback = void (*)(PowerState state);
    void setStateChangeCallback(PowerStateCallback callback);

private:
    int batteryPin;
    StorageManager* storageManager;

    // Battery parameters
    float batteryVoltage;
    float batteryCapacity;
    float batteryFullVoltage;
    float batteryEmptyVoltage;

    // Current accumulated energy usage in mAh
    float energyUsed;

    // Threshold levels
    float lowVoltageThreshold;
    float criticalVoltageThreshold;

    // Current power state
    PowerState currentState;

    // Callback for state changes
    PowerStateCallback stateChangeCallback;

    // Calculate battery voltage from ADC reading
    float calculateBatteryVoltage(int adcReading);

    // Time tracking
    unsigned long lastUpdateTime;
    unsigned long lastLogTime;

    // ADC configuration
    //    static const int ADC_RESOLUTION = 4096;  // 12-bit ADC

    // Voltage calculation
    float referenceVoltage;       // ADC reference voltage
    float voltageDividerRatio;    // For voltage divider if used
};

#endif // POWER_MANAGER_H