/**
 * State Handlers - Implementation
 */

#include "StateHandlers.h"
#include "Config.h"
#include "PinDefinitions.h"

// Initialize static members
unsigned long StateHandlers::lastSensorUpdateTime = 0;
unsigned long StateHandlers::lastTelemetryTime = 0;
unsigned long StateHandlers::flightStartTime = 0;

void StateHandlers::setupHandlers(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        DiagnosticManager* diagnosticManager,
        PreflightCheckSystem* preflightSystem,
        PowerManager* powerManager
) {
    // Register state handlers
    stateMachine.registerStateHandler(RocketState::INIT, [&stateMachine, dataManager, loraSystem, storageManager, diagnosticManager, preflightSystem]() {
        handleInitState(stateMachine, dataManager, loraSystem, storageManager, diagnosticManager, preflightSystem);
    });

    stateMachine.registerStateHandler(RocketState::GROUND_IDLE, [&stateMachine, dataManager, loraSystem, storageManager, powerManager]() {
        handleGroundIdleState(stateMachine, dataManager, loraSystem, storageManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::READY, [&stateMachine, dataManager, loraSystem, storageManager, diagnosticManager, powerManager]() {
        handleReadyState(stateMachine, dataManager, loraSystem, storageManager, diagnosticManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::POWERED_FLIGHT, [&stateMachine, dataManager, loraSystem, storageManager, powerManager]() {
        handlePoweredFlightState(stateMachine, dataManager, loraSystem, storageManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::COASTING, [&stateMachine, dataManager, loraSystem, storageManager, powerManager]() {
        handleCoastingState(stateMachine, dataManager, loraSystem, storageManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::APOGEE, [&stateMachine, dataManager, loraSystem, storageManager]() {
        handleApogeeState(stateMachine, dataManager, loraSystem, storageManager);
    });

    stateMachine.registerStateHandler(RocketState::DESCENT, [&stateMachine, dataManager, loraSystem, storageManager]() {
        handleDescentState(stateMachine, dataManager, loraSystem, storageManager);
    });

    stateMachine.registerStateHandler(RocketState::PARACHUTE_DESCENT, [&stateMachine, dataManager, loraSystem, storageManager, powerManager]() {
        handleParachuteDescentState(stateMachine, dataManager, loraSystem, storageManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::LANDED, [&stateMachine, dataManager, loraSystem, storageManager, powerManager]() {
        handleLandedState(stateMachine, dataManager, loraSystem, storageManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::ERROR, [&stateMachine, dataManager, loraSystem, storageManager]() {
        handleErrorState(stateMachine, dataManager, loraSystem, storageManager);
    });

    stateMachine.registerEventHandler(RocketState::GROUND_IDLE, [&stateMachine, dataManager, loraSystem, storageManager](RocketEvent event) {
        return handleGroundIdleEvents(event, stateMachine, dataManager, loraSystem, storageManager);
    });

    // Initialize timekeeping
    lastSensorUpdateTime = millis();
    lastTelemetryTime = millis();
}

void StateHandlers::handleInitState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        DiagnosticManager* diagnosticManager,
        PreflightCheckSystem* preflightSystem
) {
    // Check the current substate
    auto subState = static_cast<InitSubState*>(stateMachine.getCurrentSubState());

    if (!subState) {
        // Default to HARDWARE_INIT if no substate
        stateMachine.processEvent(RocketEvent::ERROR_DETECTED);
        return;
    }

    switch (*subState) {
        case InitSubState::HARDWARE_INIT:
            // Initialize hardware and move to next substate
            if (storageManager) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Initializing hardware...");
            }

            // Move to sensor calibration
            *subState = InitSubState::SENSOR_CALIBRATION;
            break;

        case InitSubState::SENSOR_CALIBRATION:
            Serial.println("Calibrating sensors...");
            // Calibrate sensors
            if (dataManager) {
                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Calibrating sensors...");
                }

                // Perform calibration
                if (dataManager->calibrateSensors()) {
                    Serial.println("Sensor calibration successful");
                    if (storageManager) {
                        storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Sensor calibration successful");
                    }
                } else {
                    Serial.println("Sensor calibration failed");
                    if (storageManager) {
                        storageManager->logMessage(LogLevel::ERROR, Subsystem::STATE_MACHINE, "Sensor calibration failed");
                    }
                    stateMachine.processEvent(RocketEvent::ERROR_DETECTED);
                    return;
                }

                // Move to SELF_TEST
                *subState = InitSubState::SELF_TEST;
            }
            break;
        case InitSubState::SELF_TEST:
            // Perform self-test of all systems
            if (storageManager) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Performing self-test...");
            }

            // Check if all subsystems are operational
            bool allSystemsGo = true;

            // Check if the DataIntegrationManager is operational
            if (!dataManager) {
                allSystemsGo = false;
                if (storageManager) {
                    storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM, "Data integration manager not available");
                }
                stateMachine.processEvent(RocketEvent::ERROR_DETECTED);
            }

            // If all systems are go, complete initialization
            if (allSystemsGo) {
                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Initialization complete");
                }

                if (preflightSystem) {
                    PreflightStatus status = preflightSystem->runPreflightChecks();

                    // Only advance to GROUND_IDLE if checks passed or had warnings
                    if (status == PreflightStatus::PASSED || status == PreflightStatus::WARNING) {
                        stateMachine.processEvent(RocketEvent::BOOT_COMPLETED);
                    } else {
                        // Move to ERROR state if critical tests failed
                        stateMachine.processEvent(RocketEvent::ERROR_DETECTED);

                        Serial.println("Pre-flight checks failed!");

                        if (storageManager) {
                            String report = preflightSystem->generatePreflightReport();

                            Serial.println(report);
                            // Log the report in chunks if necessary
                            // (limited by log message size)
                            int chunkSize = 60;
                            for (int i = 0; i < report.length(); i += chunkSize) {
                                String chunk = report.substring(i, min(i + chunkSize, report.length()));
                                storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM, chunk.c_str());
                            }
                        }
                    }
                } else {
                    Serial.println("Pre-flight system not available, skipping checks.");
                    // No pre-flight system, advance anyway
                    stateMachine.processEvent(RocketEvent::BOOT_COMPLETED);
                }
            } else {
                if (storageManager) {
                    storageManager->logMessage(LogLevel::ERROR, Subsystem::STATE_MACHINE, "Self-test failed");
                }

                // Signal error
                stateMachine.processEvent(RocketEvent::ERROR_DETECTED);
            }
            break;
    }

    // Handle LED indicators
    static unsigned long lastLedTime = 0;
    unsigned long currentTime = millis(); // Asegúrate de declarar currentTime
    if (currentTime - lastLedTime >= 500) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
}

void StateHandlers::handleGroundIdleState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        PowerManager* powerManager
) {
    // Check the current substate
    auto subState = static_cast<GroundIdleSubState*>(stateMachine.getCurrentSubState());
    if (!subState) {
        // Default to SLEEP if no substate
        subState = new GroundIdleSubState(GroundIdleSubState::SLEEP);
        if (storageManager) {
            storageManager->logMessage(LogLevel::ERROR, Subsystem::STATE_MACHINE, "No substate found, defaulting to SLEEP");
        }
    }

    // Update sensors at a reduced rate to save power
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= GROUND_IDLE_SENSOR_RATE) {  // 1Hz updates in idle
        lastSensorUpdateTime = currentTime;

        if (dataManager){
            // Update data integration system
            dataManager->update();
        }
    }

    // Periodically send telemetry
    if (currentTime - lastTelemetryTime >= GROUND_IDLE_TELEMETRY_RATE) {  // 0.2Hz telemetry in idle
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    dataManager,
                    powerManager,
                    static_cast<uint8_t>(RocketState::GROUND_IDLE)
            );
        }else{
            Serial.println("LoRa system not available, cannot send telemetry.");
        }
    }

    switch (*subState) {
        case GroundIdleSubState::SLEEP:
            // In deep sleep mode, waiting for wake command
            // This would be implemented with actual sleep mode in production
            break;

        case GroundIdleSubState::RECEIVING_COMMANDS:
            // Actively checking for commands
            break;

        case GroundIdleSubState::LOW_POWER:
            // Low power monitoring mode
            break;
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 250) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }

    //TODO: Change the state to READY if a launch command is received (LoRa cant get into receive, dont know why)
    stateMachine.processEvent(RocketEvent::WAKE_UP_COMMAND);
}

void StateHandlers::handleReadyState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        DiagnosticManager* diagnosticManager,
        PowerManager* powerManager
) {
//    Serial.println("Ready state handler called");
    // Update sensors at a higher rate now that we're armed
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= READY_SENSOR_RATE) {  // 10Hz updates when ready
        lastSensorUpdateTime = currentTime;

        if (dataManager) {
            dataManager->update();
        }

        // Log sensor data
        if (storageManager && dataManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry{};
            FlightData flightData = dataManager->getFlightData();

            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::POWERED_FLIGHT);
            telemetry.altitude = flightData.altitude;
            telemetry.vertSpeed = flightData.verticalSpeed;
            telemetry.accelX = flightData.accelData.x;
            telemetry.accelY = flightData.accelData.y;
            telemetry.accelZ = flightData.accelData.z;
            telemetry.gyroX = flightData.gyroData.x;
            telemetry.gyroY = flightData.gyroData.y;
            telemetry.gyroZ = flightData.gyroData.z;
            telemetry.temperature = flightData.temperature;
            telemetry.pressure = flightData.pressure;
            telemetry.batteryMv = flightData.batteryVoltage;
            telemetry.gpsLat = flightData.gpsData.latitude;
            telemetry.gpsLon = flightData.gpsData.longitude;
            telemetry.gpsAlt = flightData.gpsData.altitude;
            telemetry.gpsSats = flightData.gpsSatellites;
            telemetry.flags = 0;
            // Set flags based on flight data
            if (flightData.apogeeDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
            }
            if (flightData.landingDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_LANDED;
            }

            storageManager->storeTelemetry(telemetry);
        }

        // Check for launch based on acceleration
        if (detectLaunch(dataManager)) {
            if (storageManager) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE,
                                           "Launch detected from acceleration!");
            }

            // Record flight start time
            flightStartTime = currentTime;

            // Trigger launch event
            stateMachine.processEvent(RocketEvent::ACCELERATION_DETECTED);
        }
    }

    // Periodically send telemetry
    if (currentTime - lastTelemetryTime >= READY_TELEMETRY_RATE) {  // 1Hz telemetry when ready
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    dataManager,
                    powerManager,
                    static_cast<uint8_t>(RocketState::READY)
            );
        }
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 100) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
}

void StateHandlers::handlePoweredFlightState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        PowerManager* powerManager
) {
    // Update sensors at full rate during flight
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= FLIGHT_SENSOR_RATE) {  // 100Hz during powered flight
        lastSensorUpdateTime = currentTime;

        if (dataManager) {
            dataManager->update();
        }

        // Log sensor data
        if (storageManager && dataManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry{};
            FlightData flightData = dataManager->getFlightData();

            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::POWERED_FLIGHT);
            telemetry.altitude = flightData.altitude;
            telemetry.vertSpeed = flightData.verticalSpeed;
            telemetry.accelX = flightData.accelData.x;
            telemetry.accelY = flightData.accelData.y;
            telemetry.accelZ = flightData.accelData.z;
            telemetry.gyroX = flightData.gyroData.x;
            telemetry.gyroY = flightData.gyroData.y;
            telemetry.gyroZ = flightData.gyroData.z;
            telemetry.temperature = flightData.temperature;
            telemetry.pressure = flightData.pressure;
            telemetry.batteryMv = flightData.batteryVoltage;
            telemetry.gpsLat = flightData.gpsData.latitude;
            telemetry.gpsLon = flightData.gpsData.longitude;
            telemetry.gpsAlt = flightData.gpsData.altitude;
            telemetry.gpsSats = flightData.gpsSatellites;
            telemetry.flags = 0;
            // Set flags based on flight data
            if (flightData.apogeeDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
            }
            if (flightData.landingDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_LANDED;
            }

            storageManager->storeTelemetry(telemetry);
        }

        // Check for burnout using flight data
        if (dataManager) {
            FlightData flightData = dataManager->getFlightData();
            static bool highAccelPhase = false;

            // First make sure we've seen high acceleration
            if (!highAccelPhase && flightData.accelData.magnitude > 3.0f) {
                highAccelPhase = true;
            }

            // Then detect when it drops below threshold
            if (highAccelPhase && flightData.accelData.magnitude < BURNOUT_ACCEL_THRESHOLD) {
                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Engine burnout detected");
                }

                stateMachine.processEvent(RocketEvent::ENGINE_BURNOUT);
            }
        }
    }

    // Send telemetry less frequently
    if (currentTime - lastTelemetryTime >= FLIGHT_TELEMETRY_RATE) {  // 5Hz telemetry during flight
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    dataManager,
                    powerManager,
                    static_cast<uint8_t>(RocketState::POWERED_FLIGHT)
            );
        }
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 50) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
}

void StateHandlers::handleCoastingState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        PowerManager* powerManager
) {
    // Update sensors at full rate during flight
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= COAST_SENSOR_RATE) {  // 100Hz during coast
        lastSensorUpdateTime = currentTime;

        if (dataManager) {
            dataManager->update();
        }

        // Log sensor data
        if (storageManager && dataManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry{};
            FlightData flightData = dataManager->getFlightData();

            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::POWERED_FLIGHT);
            telemetry.altitude = flightData.altitude;
            telemetry.vertSpeed = flightData.verticalSpeed;
            telemetry.accelX = flightData.accelData.x;
            telemetry.accelY = flightData.accelData.y;
            telemetry.accelZ = flightData.accelData.z;
            telemetry.gyroX = flightData.gyroData.x;
            telemetry.gyroY = flightData.gyroData.y;
            telemetry.gyroZ = flightData.gyroData.z;
            telemetry.temperature = flightData.temperature;
            telemetry.pressure = flightData.pressure;
            telemetry.batteryMv = flightData.batteryVoltage;
            telemetry.gpsLat = flightData.gpsData.latitude;
            telemetry.gpsLon = flightData.gpsData.longitude;
            telemetry.gpsAlt = flightData.gpsData.altitude;
            telemetry.gpsSats = flightData.gpsSatellites;
            telemetry.flags = 0;
            // Set flags based on flight data
            if (flightData.apogeeDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
            }
            if (flightData.landingDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_LANDED;
            }

            storageManager->storeTelemetry(telemetry);
        }

        // Check for apogee
        if (detectApogee(dataManager)) {
            if (storageManager) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Apogee detected!");
            }

            stateMachine.processEvent(RocketEvent::APOGEE_DETECTED);
        }
    }

    // Send telemetry
    if (currentTime - lastTelemetryTime >= COAST_TELEMETRY_RATE) {  // 5Hz telemetry
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    dataManager,
                    powerManager,
                    static_cast<uint8_t>(RocketState::COASTING)
            );
        }
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 50) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
}

void StateHandlers::handleApogeeState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager
) {
    // The apogee state is a transient state - immediately go to descent
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "At apogee, transitioning to descent");
    }

    // Force transition to descent
    stateMachine.processEvent(RocketEvent::APOGEE_DETECTED);
}

void StateHandlers::handleDescentState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager
) {
    // Deploy parachute
    deployParachute();

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::PARACHUTE, "Parachute deployed");
    }

    // Immediately transition to parachute descent
    stateMachine.processEvent(RocketEvent::PARACHUTE_DEPLOYED);
}

void StateHandlers::handleParachuteDescentState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        PowerManager* powerManager
) {
    // Update sensors at a moderate rate during descent
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= DESCENT_SENSOR_RATE) {  // 50Hz during descent
        lastSensorUpdateTime = currentTime;

        if (dataManager) {
            dataManager->update();
        }

        // Log sensor data
        if (storageManager && dataManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry{};
            FlightData flightData = dataManager->getFlightData();

            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::POWERED_FLIGHT);
            telemetry.altitude = flightData.altitude;
            telemetry.vertSpeed = flightData.verticalSpeed;
            telemetry.accelX = flightData.accelData.x;
            telemetry.accelY = flightData.accelData.y;
            telemetry.accelZ = flightData.accelData.z;
            telemetry.gyroX = flightData.gyroData.x;
            telemetry.gyroY = flightData.gyroData.y;
            telemetry.gyroZ = flightData.gyroData.z;
            telemetry.temperature = flightData.temperature;
            telemetry.pressure = flightData.pressure;
            telemetry.batteryMv = flightData.batteryVoltage;
            telemetry.gpsLat = flightData.gpsData.latitude;
            telemetry.gpsLon = flightData.gpsData.longitude;
            telemetry.gpsAlt = flightData.gpsData.altitude;
            telemetry.gpsSats = flightData.gpsSatellites;
            telemetry.flags = 0;
            // Set flags based on flight data
            if (flightData.apogeeDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
            }
            if (flightData.landingDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_LANDED;
            }

            storageManager->storeTelemetry(telemetry);
        }

        // Check for landing
        if (detectLanding(dataManager)) {
            if (storageManager) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Landing detected!");
            }

            stateMachine.processEvent(RocketEvent::LANDING_DETECTED);
        }
    }

    // Send telemetry
    if (currentTime - lastTelemetryTime >= DESCENT_TELEMETRY_RATE) {  // 2Hz telemetry during descent
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    dataManager,
                    powerManager,
                    static_cast<uint8_t>(RocketState::PARACHUTE_DESCENT)
            );
        }
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 100) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
}

void StateHandlers::handleLandedState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        PowerManager* powerManager
) {
    // We've landed, reduce sampling rate
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= LANDED_SENSOR_RATE) {  // 2Hz after landing
        lastSensorUpdateTime = currentTime;

        if (dataManager) {
            dataManager->update();
        }

        // Log sensor data
        if (storageManager && dataManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry{};
            FlightData flightData = dataManager->getFlightData();

            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::POWERED_FLIGHT);
            telemetry.altitude = flightData.altitude;
            telemetry.vertSpeed = flightData.verticalSpeed;
            telemetry.accelX = flightData.accelData.x;
            telemetry.accelY = flightData.accelData.y;
            telemetry.accelZ = flightData.accelData.z;
            telemetry.gyroX = flightData.gyroData.x;
            telemetry.gyroY = flightData.gyroData.y;
            telemetry.gyroZ = flightData.gyroData.z;
            telemetry.temperature = flightData.temperature;
            telemetry.pressure = flightData.pressure;
            telemetry.batteryMv = flightData.batteryVoltage;
            telemetry.gpsLat = flightData.gpsData.latitude;
            telemetry.gpsLon = flightData.gpsData.longitude;
            telemetry.gpsAlt = flightData.gpsData.altitude;
            telemetry.gpsSats = flightData.gpsSatellites;
            telemetry.flags = 0;
            // Set flags based on flight data
            if (flightData.apogeeDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
            }
            if (flightData.landingDetected) {
                telemetry.flags |= TelemetryPacket::FLAG_LANDED;
            }

            storageManager->storeTelemetry(telemetry);
        }
    }

    // Send telemetry at reduced rate to save power but allow tracking
    if (currentTime - lastTelemetryTime >= LANDED_TELEMETRY_RATE) {  // 1Hz telemetry after landing
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    dataManager,
                    powerManager,
                    static_cast<uint8_t>(RocketState::LANDED)
            );
        }
    }

    // Transfer data from flash to SD if we haven't already
    static bool dataTransferred = false;
    if (!dataTransferred && storageManager) {
        dataTransferred = true;
        storageManager->transferData();
        storageManager->logMessage(LogLevel::INFO, Subsystem::STORAGE, "Flight data transferred to SD card");
    }

    // Automatically enter low power mode after landing
    static bool lowPowerEnabled = false;
    if (!lowPowerEnabled && powerManager) {
        lowPowerEnabled = true;
        powerManager->enterPowerState(PowerState::LOW_POWER);

        if (storageManager) {
            storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                       "Automatically entering low power mode after landing");
        }
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 500) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
}

void StateHandlers::handleErrorState(
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager
) {
    // Check the current substate
    auto subState = static_cast<ErrorSubState*>(stateMachine.getCurrentSubState());
    if (!subState) {
        // Default to SENSOR_ERROR if no substate
        subState = new ErrorSubState(ErrorSubState::SENSOR_ERROR);
        if (storageManager) {
            storageManager->logMessage(LogLevel::ERROR, Subsystem::STATE_MACHINE, "No substate found, defaulting to SENSOR_ERROR");
        }
    }

    // Keep sensors updated at a low rate
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= ERROR_SENSOR_RATE) {  // 1Hz in error state
        lastSensorUpdateTime = currentTime;

        if (dataManager) {
            dataManager->update();
        }
    }

    // Send error telemetry more frequently
    if (currentTime - lastTelemetryTime >= ERROR_TELEMETRY_RATE) {  // 0.5Hz telemetry in error state
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Create and send error telemetry message
            // Include the error state
            TelemetryPacket errorPacket{};
            errorPacket.timestamp = currentTime;
            errorPacket.rocketState = static_cast<uint8_t>(RocketState::ERROR);
            //TODO Data de los sensores
        }
    }

    switch (*subState) {
        case ErrorSubState::SENSOR_ERROR:
            // Handle sensor errors - try to recover
            if (dataManager) {
                // Check if all critical sensors are working
                FlightData data = dataManager->getFlightData();
                bool sensorsWorking = true;

                // Add checks for critical sensor functionality

                if (sensorsWorking) {
                    if (storageManager) {
                        storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Sensors recovered, returning to idle");
                    }
                    stateMachine.processEvent(RocketEvent::RECOVERY_SUCCEEDED);
                }
            }
            break;

        case ErrorSubState::COMMUNICATION_ERROR:
            // Handle communication errors
            if (loraSystem && loraSystem->isOperational()) {
                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Communication recovered, returning to idle");
                }

                stateMachine.processEvent(RocketEvent::RECOVERY_SUCCEEDED);
            }
            break;

        case ErrorSubState::STORAGE_ERROR:
            // Handle storage errors
            if (storageManager && storageManager->isOperational()) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Storage recovered, returning to idle");

                stateMachine.processEvent(RocketEvent::RECOVERY_SUCCEEDED);
            }
            break;

        case ErrorSubState::BATTERY_LOW:
            // Handle low battery - not much we can do except wait for voltage to recover
            break;

        case ErrorSubState::RECOVERY_MODE:
            // In recovery mode, waiting for commands
            break;
    }

    static unsigned long lastLedTime = 0;
    if (currentTime - lastLedTime >= 100) { // Cambia el intervalo según el estado
        lastLedTime = currentTime;
        digitalWrite(LED_RED, !digitalRead(LED_RED));
    }
}

bool StateHandlers::handleGroundIdleEvents(
        RocketEvent event,
        StateMachine& stateMachine,
        DataIntegrationManager* dataManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager
) {
    Serial.print("DEBUG: Ground Idle processing event: ");
    Serial.println(static_cast<int>(event));

    // Handle ground idle-specific events
    switch (event) {
        case RocketEvent::WAKE_UP_COMMAND:
            Serial.println("DEBUG: Processing WAKE_UP_COMMAND");
            // Change substate to receiving commands
            auto subState = static_cast<GroundIdleSubState*>(stateMachine.getCurrentSubState());
            if (subState) {
                *subState = GroundIdleSubState::RECEIVING_COMMANDS;

                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Received wake up command");
                }

                Serial.println("DEBUG: Wake up command processed successfully");
                return true;
            }
            break;
    }

    Serial.println("DEBUG: Event not handled by Ground Idle handler");
    return false;  // Event not handled
}

void StateHandlers::deployParachute() {
    // Hardware-specific code to deploy the parachute
    // For example, activating a GPIO pin or controlling a servo
}

bool StateHandlers::detectLaunch(DataIntegrationManager* dataManager) {
    if (!dataManager) return false;

    // Launch is detected when vertical acceleration exceeds threshold
    // and vertical speed is positive
    FlightData flightData = dataManager->getFlightData();
    return (flightData.verticalAccel > LAUNCH_ACCELERATION_THRESHOLD * 9.81f) &&
           (flightData.verticalSpeed > 1.0f);
}

bool StateHandlers::detectApogee(DataIntegrationManager* dataManager) {
    if (!dataManager) return false;

    return dataManager->isApogeeDetected();
}

bool StateHandlers::detectLanding(DataIntegrationManager* dataManager) {
    if (!dataManager) return false;

    return dataManager->isLandingDetected();
}

void StateHandlers::sendTelemetryData(
        LoRaSystem* loraSystem,
        DataIntegrationManager* dataManager,
        PowerManager* powerManager,
        uint8_t rocketState
) {
    if (!loraSystem) {
        Serial.println("LoRa system not available, cannot send telemetry.");
        return;
    }

    // Create telemetry packet
    TelemetryPacket packet{};

    // Set timestamp and state
    packet.timestamp = millis();
    packet.rocketState = rocketState;

//    Serial.println("State: " + String(rocketState));

    // Get sensor data from DataIntegrationManager
    if (dataManager) {
        FlightData flightData = dataManager->getFlightData();
        packet.altitude = flightData.altitude;
        packet.verticalSpeed = flightData.verticalSpeed;
        packet.acceleration = flightData.accelData.magnitude;
        packet.temperature = flightData.temperature;
        packet.pressure = flightData.pressure;
        packet.gpsSatellites = flightData.gpsSatellites;
        packet.gpsLatitude = flightData.gpsData.latitude;
        packet.gpsLongitude = flightData.gpsData.longitude;
        packet.gpsAltitude = flightData.gpsData.altitude;

        // Set flags if apogee detected
        if (flightData.apogeeDetected) {
            packet.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
        }
        // Set flags if descent detected
        if (flightData.landingDetected) {
            packet.flags |= TelemetryPacket::FLAG_LANDED;
        }
    } else {
        // Default values if no data manager available
        packet.altitude = 0.0f;
        packet.verticalSpeed = 0.0f;
        packet.acceleration = 0.0f;
        packet.temperature = 0.0f;
        packet.pressure = 0.0f;
        packet.gpsSatellites = 0;
        packet.gpsLatitude = 0.0f;
        packet.gpsLongitude = 0.0f;
        packet.gpsAltitude = 0.0f;
    }

    // Battery data
    packet.batteryVoltage = powerManager ? powerManager->getBatteryVoltage() : 0.0f;

    // Check for low battery
    if (packet.batteryVoltage < 3.5f) {
        packet.flags |= TelemetryPacket::FLAG_LOW_BATTERY;
    }

    // Set parachute flag if in parachute descent state
    if (rocketState == static_cast<uint8_t>(RocketState::PARACHUTE_DESCENT)) {
        packet.flags |= TelemetryPacket::FLAG_PARACHUTE_DEPLOYED;
    }

    // Set landed flag if in landed state
    if (rocketState == static_cast<uint8_t>(RocketState::LANDED)) {
        packet.flags |= TelemetryPacket::FLAG_LANDED;
    }

    // Set error flag if in error state
    if (rocketState == static_cast<uint8_t>(RocketState::ERROR)) {
        packet.flags |= TelemetryPacket::FLAG_ERROR_CONDITION;
    }

    // Create sensor status byte
    packet.sensorStatus = 0;

    // Now serialize and send the packet
    TelemetrySerializer serializer;
    std::vector<uint8_t> packetData = serializer.serialize(packet);

    // Create LoRa message
    Message message;
    message.type = MessageType::TELEMETRY;
    message.priority = 100;  // Medium priority for telemetry
    message.timestamp = millis();
    message.length = packetData.size();
    message.data = new uint8_t[message.length];
    memcpy(message.data, packetData.data(), message.length);
    message.acknowledged = false;

    // Send the message
    bool result = loraSystem->sendMessage(message);

    if (!result) {
        Serial.println("Failed to send telemetry data");
    }

    // Clean up
    delete[] message.data;
}