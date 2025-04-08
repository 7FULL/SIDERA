/**
 * Power Controller
 *
 * Implements power saving strategies based on battery state
 */

#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include "PowerManager.h"
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../StateMachine/StateMachine.h"

class PowerController {
public:
    PowerController(
            PowerManager* powerManager,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StateMachine* stateMachine
    );

    // Initialize power controller
    bool begin();

    // Update power management strategy based on system state
    void update();

    // Apply power saving mode
    void applyPowerSaving(PowerState state);

    // Configure update rates for different power states
    void setUpdateRates(int normalRate, int lowPowerRate, int criticalRate);

    // Configure telemetry rates for different power states
    void setTelemetryRates(int normalRate, int lowPowerRate, int criticalRate);

private:
    // Subsystem references
    PowerManager* powerManager;
    BarometricSensorManager* baroManager;
    IMUSensorManager* imuManager;
    GPSSensorManager* gpsManager;
    LoRaSystem* loraSystem;
    StateMachine* stateMachine;

    // Update rates (ms) for sensor sampling
    int normalUpdateRate;
    int lowPowerUpdateRate;
    int criticalUpdateRate;

    // Telemetry rates (ms) for data transmission
    int normalTelemetryRate;
    int lowPowerTelemetryRate;
    int criticalTelemetryRate;

    // Current update and telemetry intervals
    int currentUpdateInterval;
    int currentTelemetryInterval;

    // Last update and telemetry times
    unsigned long lastSensorUpdateTime;
    unsigned long lastTelemetryTime;

    // Current power state
    PowerState currentPowerState;

    // Power state callback
    static void onPowerStateChange(PowerState state);
    static PowerController* instance; // For callback

    // Apply flight-phase specific power strategies
    void applyIdlePowerStrategy(PowerState state);
    void applyFlightPowerStrategy(PowerState state);
    void applyRecoveryPowerStrategy(PowerState state);

    // Power control for specific subsystems
    void adjustSensorSampling(PowerState state);
    void adjustTelemetryRate(PowerState state);
    void adjustGPSPower(PowerState state);
};

#endif // POWER_CONTROLLER_H