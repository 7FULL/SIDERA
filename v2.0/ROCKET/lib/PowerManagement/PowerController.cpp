/**
 * Power Controller Implementation
 */

#include "PowerController.h"

// Initialize static member
PowerController* PowerController::instance = nullptr;

PowerController::PowerController(
        PowerManager* powerManager,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StateMachine* stateMachine
)
        : powerManager(powerManager),
          baroManager(baroManager),
          imuManager(imuManager),
          gpsManager(gpsManager),
          loraSystem(loraSystem),
          stateMachine(stateMachine),
          normalUpdateRate(10),       // 10ms (100Hz) in normal mode
          lowPowerUpdateRate(20),     // 20ms (50Hz) in low power mode
          criticalUpdateRate(100),    // 100ms (10Hz) in critical mode
          normalTelemetryRate(200),   // 200ms (5Hz) in normal mode
          lowPowerTelemetryRate(1000),// 1000ms (1Hz) in low power mode
          criticalTelemetryRate(5000),// 5000ms (0.2Hz) in critical mode
          currentUpdateInterval(normalUpdateRate),
          currentTelemetryInterval(normalTelemetryRate),
          lastSensorUpdateTime(0),
          lastTelemetryTime(0),
          currentPowerState(PowerState::NORMAL)
{
    // Save instance for callback
    instance = this;
}

bool PowerController::begin() {
    if (!powerManager) {
        return false;
    }

    // Register for power state changes
    powerManager->setStateChangeCallback(onPowerStateChange);

    // Initialize with current power state
    currentPowerState = powerManager->getPowerState();
    applyPowerSaving(currentPowerState);

    lastSensorUpdateTime = millis();
    lastTelemetryTime = lastSensorUpdateTime;

    return true;
}

void PowerController::update() {
    if (!powerManager) {
        return;
    }

    // Get current time
    unsigned long currentTime = millis();

    // Check if it's time to update sensor sampling
    if (currentTime - lastSensorUpdateTime >= currentUpdateInterval) {
        lastSensorUpdateTime = currentTime;

        // Update sensors based on current interval
        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();

        // GPS updates are handled separately based on power state
    }

    // Check if it's time to send telemetry
    if (currentTime - lastTelemetryTime >= currentTelemetryInterval) {
        lastTelemetryTime = currentTime;

        // Send telemetry based on current interval
        // This would call your telemetry transmission code
    }

    // Apply power saving strategy based on current rocket state
    if (stateMachine) {
        RocketState rocketState = stateMachine->getCurrentState();

        // Apply different strategies based on flight phase
        if (rocketState == RocketState::GROUND_IDLE ||
            rocketState == RocketState::READY ||
            rocketState == RocketState::INIT) {

            applyIdlePowerStrategy(currentPowerState);
        }
        else if (rocketState == RocketState::POWERED_FLIGHT ||
                 rocketState == RocketState::COASTING ||
                 rocketState == RocketState::APOGEE ||
                 rocketState == RocketState::DESCENT ||
                 rocketState == RocketState::PARACHUTE_DESCENT) {

            applyFlightPowerStrategy(currentPowerState);
        }
        else if (rocketState == RocketState::LANDED) {
            applyRecoveryPowerStrategy(currentPowerState);
        }
    }
}

void PowerController::applyPowerSaving(PowerState state) {
    currentPowerState = state;

    // Apply power saving measures based on state
    switch (state) {
        case PowerState::NORMAL:
            currentUpdateInterval = normalUpdateRate;
            currentTelemetryInterval = normalTelemetryRate;
            break;

        case PowerState::LOW_POWER:
            currentUpdateInterval = lowPowerUpdateRate;
            currentTelemetryInterval = lowPowerTelemetryRate;
            break;

        case PowerState::CRITICAL:
            currentUpdateInterval = criticalUpdateRate;
            currentTelemetryInterval = criticalTelemetryRate;
            break;
    }

    // Apply subsystem-specific adjustments
    adjustSensorSampling(state);
    adjustTelemetryRate(state);
    adjustGPSPower(state);
}

void PowerController::setUpdateRates(int normalRate, int lowPowerRate, int criticalRate) {
    normalUpdateRate = normalRate;
    lowPowerUpdateRate = lowPowerRate;
    criticalUpdateRate = criticalRate;

    // Update current interval if needed
    if (currentPowerState == PowerState::NORMAL) {
        currentUpdateInterval = normalRate;
    } else if (currentPowerState == PowerState::LOW_POWER) {
        currentUpdateInterval = lowPowerRate;
    } else {
        currentUpdateInterval = criticalRate;
    }
}

void PowerController::setTelemetryRates(int normalRate, int lowPowerRate, int criticalRate) {
    normalTelemetryRate = normalRate;
    lowPowerTelemetryRate = lowPowerRate;
    criticalTelemetryRate = criticalRate;

    // Update current interval if needed
    if (currentPowerState == PowerState::NORMAL) {
        currentTelemetryInterval = normalRate;
    } else if (currentPowerState == PowerState::LOW_POWER) {
        currentTelemetryInterval = lowPowerRate;
    } else {
        currentTelemetryInterval = criticalRate;
    }
}

void PowerController::onPowerStateChange(PowerState state) {
    // Static callback for power manager
    if (instance) {
        instance->applyPowerSaving(state);
    }
}

void PowerController::applyIdlePowerStrategy(PowerState state) {
    // In idle state, we can be more aggressive with power saving
    switch (state) {
        case PowerState::NORMAL:
            // Use standard rates but enable GPS low power when appropriate
            if (gpsManager) {
                // Only enable GPS periodically
                static unsigned long lastGpsTime = 0;
                unsigned long currentTime = millis();

                if (currentTime - lastGpsTime >= 10000) { // 10 second cycle
                    lastGpsTime = currentTime;

                    // Enable GPS for 5 seconds, then sleep
                    gpsManager->disableLowPowerMode();

                    // Schedule GPS sleep after 5 seconds
                    delay(5000);
                    gpsManager->enableLowPowerMode();
                }
            }
            break;

        case PowerState::LOW_POWER:
            // Reduce rates further and put GPS in low power mode
            currentUpdateInterval = lowPowerUpdateRate * 2; // Even slower updates
            currentTelemetryInterval = lowPowerTelemetryRate * 2; // Slower telemetry

            if (gpsManager) {
                gpsManager->enableLowPowerMode();
            }
            break;

        case PowerState::CRITICAL:
            // Minimal operations
            currentUpdateInterval = criticalUpdateRate * 2;
            currentTelemetryInterval = criticalTelemetryRate * 2;

            // Disable GPS completely
            if (gpsManager) {
                gpsManager->enableLowPowerMode();
            }

            // Minimize LoRa transmissions
            // Only send status messages every 30 seconds
            break;
    }
}

void PowerController::applyFlightPowerStrategy(PowerState state) {
    // During flight, prioritize sensor accuracy and critical systems
    switch (state) {
        case PowerState::NORMAL:
            // Full power to all systems during flight
            currentUpdateInterval = normalUpdateRate;
            currentTelemetryInterval = normalTelemetryRate;

            if (gpsManager) {
                gpsManager->disableLowPowerMode();
            }
            break;

        case PowerState::LOW_POWER:
            // Maintain critical sensor rates but reduce telemetry
            currentUpdateInterval = normalUpdateRate; // Keep sensor rates high
            currentTelemetryInterval = lowPowerTelemetryRate; // Reduce telemetry

            // Keep GPS on for landing location
            if (gpsManager) {
                gpsManager->disableLowPowerMode();
            }
            break;

        case PowerState::CRITICAL:
            // Focus only on critical flight parameters
            // Maintain decent sensor rate but minimal telemetry
            currentUpdateInterval = lowPowerUpdateRate; // Reduced but still adequate
            currentTelemetryInterval = criticalTelemetryRate; // Minimal telemetry

            // Reduce GPS power but keep active
            // GPS is important for recovery
            if (gpsManager) {
                // Cycle GPS power to save energy but still get position
                static unsigned long lastGpsTime = 0;
                unsigned long currentTime = millis();

                if (currentTime - lastGpsTime >= 5000) { // 5 second cycle
                    lastGpsTime = currentTime;

                    // Enable GPS for 2 seconds, then sleep
                    gpsManager->disableLowPowerMode();

                    // Schedule GPS sleep
                    delay(2000);
                    gpsManager->enableLowPowerMode();
                }
            }
            break;
    }
}

void PowerController::applyRecoveryPowerStrategy(PowerState state) {
    // In recovery mode, prioritize GPS and telemetry for finding the rocket
    switch (state) {
        case PowerState::NORMAL:
            // Focus on GPS and telemetry, reduce other sensors
            currentUpdateInterval = lowPowerUpdateRate;
            currentTelemetryInterval = normalTelemetryRate; // High telemetry rate

            if (gpsManager) {
                gpsManager->disableLowPowerMode(); // Full GPS power
            }
            break;

        case PowerState::LOW_POWER:
            // Further reduce sensor sampling, preserve GPS and periodic telemetry
            currentUpdateInterval = lowPowerUpdateRate * 2;
            currentTelemetryInterval = lowPowerTelemetryRate;

            if (gpsManager) {
                // Cycle GPS to save power
                static unsigned long lastGpsTime = 0;
                unsigned long currentTime = millis();

                if (currentTime - lastGpsTime >= 30000) { // 30 second cycle
                    lastGpsTime = currentTime;

                    // Enable GPS for 10 seconds, then sleep
                    gpsManager->disableLowPowerMode();

                    // Schedule GPS sleep
                    delay(10000);
                    gpsManager->enableLowPowerMode();
                }
            }
            break;

        case PowerState::CRITICAL:
            // Minimal operation - GPS fix and occasional telemetry
            currentUpdateInterval = criticalUpdateRate * 2;
            currentTelemetryInterval = criticalTelemetryRate * 2;

            if (gpsManager) {
                // Get GPS fix once per minute
                static unsigned long lastGpsTime = 0;
                unsigned long currentTime = millis();

                if (currentTime - lastGpsTime >= 60000) { // 60 second cycle
                    lastGpsTime = currentTime;

                    // Enable GPS for 10 seconds, then sleep
                    gpsManager->disableLowPowerMode();

                    // Schedule GPS sleep
                    delay(10000);
                    gpsManager->enableLowPowerMode();
                }
            }
            break;
    }
}

void PowerController::adjustSensorSampling(PowerState state) {
    // Adjust sensor sampling based on power state
    // This complements the interval changes

    switch (state) {
        case PowerState::NORMAL:
            // Full sensor operations
            break;

        case PowerState::LOW_POWER:
            // Reduce resolution or oversampling if possible
            break;

        case PowerState::CRITICAL:
            // Minimal sensor operations
            break;
    }
}

void PowerController::adjustTelemetryRate(PowerState state) {
    // Adjust telemetry content and rate based on power state
    if (!loraSystem) {
        return;
    }

    switch (state) {
        case PowerState::NORMAL:
            // Full telemetry data
            // No adjustments needed
            break;

        case PowerState::LOW_POWER:
            // Reduce transmission power if possible
            loraSystem->setTxPower(14); // Reduce from default 17dBm
            break;

        case PowerState::CRITICAL:
            // Minimal transmission power
            loraSystem->setTxPower(10); // Minimum viable power
            break;
    }
}

void PowerController::adjustGPSPower(PowerState state) {
    // Adjust GPS power based on state
    if (!gpsManager) {
        return;
    }

    switch (state) {
        case PowerState::NORMAL:
            // Full GPS operation
            gpsManager->disableLowPowerMode();
            break;

        case PowerState::LOW_POWER:
            // Depends on rocket state (handled in specific strategies)
            break;

        case PowerState::CRITICAL:
            // Depends on rocket state (handled in specific strategies)
            break;
    }
}