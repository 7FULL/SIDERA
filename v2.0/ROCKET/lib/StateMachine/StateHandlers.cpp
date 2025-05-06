/**
 * State Handlers - Implementation
 */

#include "StateHandlers.h"
#include "Config.h"

// Initialize static members
unsigned long StateHandlers::lastSensorUpdateTime = 0;
unsigned long StateHandlers::lastTelemetryTime = 0;
unsigned long StateHandlers::flightStartTime = 0;

void StateHandlers::setupHandlers(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        DiagnosticManager* diagnosticManager,
        PreflightCheckSystem* preflightSystem,
        PowerManager* powerManager
) {
    // Register state handlers
    stateMachine.registerStateHandler(RocketState::INIT, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, diagnosticManager, preflightSystem]() {
        handleInitState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, diagnosticManager, preflightSystem);
    });

    stateMachine.registerStateHandler(RocketState::GROUND_IDLE, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager]() {
        handleGroundIdleState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::READY, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, diagnosticManager, powerManager]() {
        handleReadyState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem,diagnosticManager, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::POWERED_FLIGHT, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager]() {
        handlePoweredFlightState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::COASTING, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager]() {
        handleCoastingState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::APOGEE, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem]() {
        handleApogeeState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem);
    });

    stateMachine.registerStateHandler(RocketState::DESCENT, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem]() {
        handleDescentState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem);
    });

    stateMachine.registerStateHandler(RocketState::PARACHUTE_DESCENT, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager]() {
        handleParachuteDescentState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::LANDED, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager]() {
        handleLandedState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem, powerManager);
    });

    stateMachine.registerStateHandler(RocketState::ERROR, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem]() {
        handleErrorState(stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem);
    });

    stateMachine.registerEventHandler(RocketState::GROUND_IDLE, [&stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem](RocketEvent event) {
        return handleGroundIdleEvents(event, stateMachine, baroManager, imuManager, gpsManager, loraSystem, storageManager, fusionSystem);
    });

    // Initialize timekeeping
    lastSensorUpdateTime = millis();
    lastTelemetryTime = millis();
}

void StateHandlers::handleInitState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        DiagnosticManager* diagnosticManager,
        PreflightCheckSystem* preflightSystem
) {
    // Check the current substate
    auto subState = static_cast<InitSubState*>(stateMachine.getCurrentSubState());

//    Serial.println("Handling INIT state...");

    if (!subState) {
        // Default to HARDWARE_INIT if no substate
//        subState = new InitSubState(InitSubState::HARDWARE_INIT);
        stateMachine.processEvent(RocketEvent::ERROR_DETECTED);
        return;
    }

//    Serial.println("Current substate: " + String(static_cast<int>(*subState)));

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
            if (imuManager && baroManager) {
                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Calibrating sensors...");
                }

                // Calibrate IMU
                imuManager->calibrate();

                // Set reference altitude
                float currentAltitude = baroManager->getAltitude();
                Serial.println("Setting reference altitude: " + String(currentAltitude));
                if (storageManager) {
                    String message = "Setting reference altitude: " + String(currentAltitude);
                    storageManager->logMessage(LogLevel::INFO, Subsystem::BAROMETER, message.c_str());
                }
                baroManager->setReferenceAltitude(currentAltitude);

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

            if (baroManager && !baroManager->getOperationalSensorCount()) {
                allSystemsGo = false;
                if (storageManager) {
                    storageManager->logMessage(LogLevel::ERROR, Subsystem::BAROMETER, "No operational barometric sensors");
                }
                stateMachine.processEvent(RocketEvent::ERROR_DETECTED);
            }

            if (imuManager && !imuManager->getOperationalSensorCount()) {
                allSystemsGo = false;
                if (storageManager) {
                    storageManager->logMessage(LogLevel::ERROR, Subsystem::IMU, "No operational IMU sensors");
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
}

void StateHandlers::handleGroundIdleState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        PowerManager* powerManager
) {
    // Check the current substate
    auto subState = static_cast<GroundIdleSubState*>(stateMachine.getCurrentSubState());
    if (!subState) {
        // Default to SLEEP if no substate
        subState = new GroundIdleSubState(GroundIdleSubState::SLEEP);
    }

    // Update sensors at a reduced rate to save power
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= GROUND_IDLE_SENSOR_RATE) {  // 1Hz updates in idle
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();
        if (gpsManager) gpsManager->update();
    }

    // Periodically send telemetry
    if (currentTime - lastTelemetryTime >= GROUND_IDLE_TELEMETRY_RATE) {  // 0.2Hz telemetry in idle
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Send telemetry data
            sendTelemetryData(
                    loraSystem,
                    baroManager,
                    imuManager,
                    gpsManager,
                    fusionSystem,
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
}

void StateHandlers::sendTelemetryData(
        LoRaSystem* loraSystem,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        SensorFusionSystem* fusionSystem,
        PowerManager* powerManager,
        uint8_t rocketState
) {
    if (!loraSystem) {
        Serial.println("LoRa system not available, cannot send telemetry.");
        return;
    }

    // Create telemetry packet
    TelemetryPacket packet;

    // Set timestamp and state
    packet.timestamp = millis();
    packet.rocketState = rocketState;

    Serial.println("State: " + String(rocketState));

    // Get sensor data from fusion system if available, otherwise from individual sensors
    if (fusionSystem) {
        FusedFlightData fusedData = fusionSystem->getFusedData();
        packet.altitude = fusedData.altitude;
        packet.verticalSpeed = fusedData.verticalSpeed;
        packet.acceleration = fusedData.verticalAccel;

        // Set flags if apogee detected
        if (fusedData.apogeeDetected) {
            packet.flags |= TelemetryPacket::FLAG_APOGEE_DETECTED;
        }
    } else if (baroManager) {
        // Fallback to barometer data
        packet.altitude = baroManager->getAltitude();
        // Not calculating velocity without fusion system
        packet.verticalSpeed = 0.0f;
        packet.acceleration = 0.0f;
    }

    // Get temperature and pressure from barometer
    if (baroManager) {
        packet.temperature = baroManager->getTemperature();
        packet.pressure = baroManager->getPressure();
    }

    // Get IMU data
    if (imuManager) {
        AccelerometerData accelData = imuManager->getAccelerometerData();
        packet.acceleration = accelData.magnitude;
    }

    // Get GPS data
    if (gpsManager && gpsManager->hasPositionFix()) {
        GPSData gpsData = gpsManager->getGPSData();
        packet.gpsSatellites = gpsData.satellites;
        packet.gpsLatitude = gpsData.latitude;
        packet.gpsLongitude = gpsData.longitude;
        packet.gpsAltitude = gpsData.altitude;
    } else {
        Serial.println("GPS data not available, using default values.");
        packet.gpsSatellites = 0;
        packet.gpsLatitude = 0.0f;
        packet.gpsLongitude = 0.0f;
        packet.gpsAltitude = 0.0f;
    }

    // Battery data
    packet.batteryVoltage = powerManager->getBatteryVoltage();

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
    Serial.printf("LoRa send result: %s\n", result ? "SUCCESS" : "FAILURE");

    // Clean up
    delete[] message.data;
}


void StateHandlers::handleReadyState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        DiagnosticManager* diagnosticManager,
        PowerManager* powerManager
) {
    // Update sensors at a higher rate now that we're armed
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= READY_SENSOR_RATE) {  // 10Hz updates when ready
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();
        if (gpsManager) gpsManager->update();

        // Check for launch based on acceleration
        if (detectLaunch(imuManager, fusionSystem)) {
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
                    baroManager,
                    imuManager,
                    gpsManager,
                    fusionSystem,
                    powerManager,
                    static_cast<uint8_t>(RocketState::READY)
            );
        }
    }
}

void StateHandlers::handlePoweredFlightState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        PowerManager* powerManager
) {
    // Update sensors at full rate during flight
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= FLIGHT_SENSOR_RATE) {  // 100Hz during powered flight
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();

        // Log sensor data
        if (storageManager && baroManager && imuManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry;
            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::POWERED_FLIGHT);
            telemetry.altitude = baroManager->getAltitude() * 1000; // Convert to mm

            // Add more telemetry fields...

            storageManager->storeTelemetry(telemetry);
        }

        // Check for burnout (using acceleration threshold)
        if (imuManager) {
            AccelerometerData accelData = imuManager->getAccelerometerData();

            static bool highAccelPhase = false;

            // First make sure we've seen high acceleration
            if (!highAccelPhase && accelData.magnitude > 3.0f) {
                highAccelPhase = true;
            }

            // Then detect when it drops below threshold
            if (highAccelPhase && accelData.magnitude < BURNOUT_ACCEL_THRESHOLD) {
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
                    baroManager,
                    imuManager,
                    gpsManager,
                    fusionSystem,
                    powerManager,
                    static_cast<uint8_t>(RocketState::POWERED_FLIGHT)
            );
        }
    }
}

void StateHandlers::handleCoastingState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        PowerManager* powerManager
) {
    // Update sensors at full rate during flight
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= COAST_SENSOR_RATE) {  // 100Hz during coast
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();

        // Log sensor data
        if (storageManager && baroManager && imuManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry;
            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::COASTING);
            telemetry.altitude = baroManager->getAltitude() * 1000; // Convert to mm

            // Add more telemetry fields...

            storageManager->storeTelemetry(telemetry);
        }

        // Check for apogee
        if (detectApogee(baroManager, fusionSystem)) {
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
                    baroManager,
                    imuManager,
                    gpsManager,
                    fusionSystem,
                    powerManager,
                    static_cast<uint8_t>(RocketState::COASTING)
            );
        }
    }
}

void StateHandlers::handleApogeeState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem
) {
    // The apogee state is a transient state - immediately go to descent
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "At apogee, transitioning to descent");
    }

    // Force transition to descent
    stateMachine.processEvent(RocketEvent::BOOT_COMPLETED);  // Using BOOT_COMPLETED as trigger
}

void StateHandlers::handleDescentState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem
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
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        PowerManager* powerManager
) {
    // Update sensors at a moderate rate during descent
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= DESCENT_SENSOR_RATE) {  // 50Hz during descent
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();
        if (gpsManager) gpsManager->update();

        // Log sensor data
        if (storageManager && baroManager && imuManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry;
            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::PARACHUTE_DESCENT);
            telemetry.altitude = baroManager->getAltitude() * 1000; // Convert to mm

            // Add more telemetry fields...

            storageManager->storeTelemetry(telemetry);
        }

        // Check for landing
        if (detectLanding(baroManager, fusionSystem)) {
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
                    baroManager,
                    imuManager,
                    gpsManager,
                    fusionSystem,
                    powerManager,
                    static_cast<uint8_t>(RocketState::PARACHUTE_DESCENT)
            );
        }
    }
}

void StateHandlers::handleLandedState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem,
        PowerManager* powerManager
) {
    // We've landed, reduce sampling rate
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= LANDED_SENSOR_RATE) {  // 2Hz after landing
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();
        if (gpsManager) gpsManager->update();

        // Log final sensor data
        if (storageManager && baroManager && imuManager && gpsManager) {
            // Create and store telemetry entry
            StoredTelemetry telemetry;
            telemetry.timestamp = currentTime;
            telemetry.state = static_cast<uint8_t>(RocketState::LANDED);
            telemetry.altitude = baroManager->getAltitude() * 1000; // Convert to mm

            // Add GPS position for recovery
            GPSData gpsData = gpsManager->getGPSData();
            telemetry.gpsLat = gpsData.latitude * 10000000; // Convert to 10^-7 degrees
            telemetry.gpsLon = gpsData.longitude * 10000000;

            // Add more telemetry fields...

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
                    baroManager,
                    imuManager,
                    gpsManager,
                    fusionSystem,
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
}

void StateHandlers::handleErrorState(
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem
) {
    // Check the current substate
    auto subState = static_cast<ErrorSubState*>(stateMachine.getCurrentSubState());
    if (!subState) {
        // Default to SENSOR_ERROR if no substate
        subState = new ErrorSubState(ErrorSubState::SENSOR_ERROR);
    }

    // Keep sensors updated at a low rate
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdateTime >= ERROR_SENSOR_RATE) {  // 1Hz in error state
        lastSensorUpdateTime = currentTime;

        if (baroManager) baroManager->update();
        if (imuManager) imuManager->update();
        if (gpsManager) gpsManager->update();
    }

    // Send error telemetry more frequently
    if (currentTime - lastTelemetryTime >= ERROR_TELEMETRY_RATE) {  // 0.5Hz telemetry in error state
        lastTelemetryTime = currentTime;

        if (loraSystem) {
            // Create and send error telemetry message
            // Include the error state
        }
    }

    switch (*subState) {
        case ErrorSubState::SENSOR_ERROR:
            // Handle sensor errors - try to recover
            if (baroManager && baroManager->getOperationalSensorCount() > 0 &&
                imuManager && imuManager->getOperationalSensorCount() > 0) {

                if (storageManager) {
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, "Sensors recovered, returning to idle");
                }

                stateMachine.processEvent(RocketEvent::RECOVERY_SUCCEEDED);
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
}

bool StateHandlers::handleGroundIdleEvents(
        RocketEvent event,
        StateMachine& stateMachine,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        SensorFusionSystem* fusionSystem
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

bool StateHandlers::detectLaunch(IMUSensorManager* imuManager, SensorFusionSystem* fusionSystem) {
    // Use the fusion system for more reliable launch detection
    if (fusionSystem) {
        // Launch is detected when vertical acceleration exceeds threshold
        // and vertical speed is positive
        FusedFlightData fusedData = fusionSystem->getFusedData();
        return (fusedData.verticalAccel > LAUNCH_ACCELERATION_THRESHOLD * 9.81f) &&
               (fusedData.verticalSpeed > 1.0f);
    }

    // Fallback to basic detection if fusion system is not available
    if (!imuManager) return false;

    // Get accelerometer data
    AccelerometerData accelData = imuManager->getAccelerometerData();

    // Check if acceleration exceeds threshold and calculate vertical component
    // Assuming Z is vertical in rocket frame
    float verticalAccel = accelData.z;

    // For more robust detection, check both magnitude and vertical component
    return (accelData.magnitude > LAUNCH_ACCELERATION_THRESHOLD &&
            verticalAccel > 0.8f * LAUNCH_ACCELERATION_THRESHOLD);
}

bool StateHandlers::detectApogee(BarometricSensorManager* baroManager, SensorFusionSystem* fusionSystem) {
    if (fusionSystem) {
        return fusionSystem->isApogeeDetected();
    }

    if (!baroManager) return false;

    // Simplified apogee detection - in reality, you'd use a more robust algorithm
    // that takes into account multiple samples and vertical velocity

    static float maxAltitude = 0.0f;
    static int descentCount = 0;

    float currentAltitude = baroManager->getAltitude();

    // Update max altitude
    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
        descentCount = 0;
        return false;
    }

    // Check if we're significantly below max altitude for several consecutive readings
    if (maxAltitude - currentAltitude > 2.0f) {  // At least 2m below max
        descentCount++;

        if (descentCount >= APOGEE_DETECTION_WINDOW) {
            return true;  // Apogee confirmed
        }
    } else {
        // Reset descent counter if we're not consistently descending
        descentCount = 0;
    }

    return false;
}

bool StateHandlers::detectLanding(BarometricSensorManager* baroManager, SensorFusionSystem* fusionSystem) {
    if (fusionSystem) {
        FusedFlightData fusedData = fusionSystem->getFusedData();

        // Landing is detected when altitude is stable and vertical speed is near zero
        return fusedData.verticalSpeed > -0.5f && fusedData.verticalSpeed < 0.5f &&
               fusedData.verticalAccel > -0.5f && fusedData.verticalAccel < 0.5f;
    }

    if (!baroManager) return false;

    static float landingAltitude = 0.0f;
    static unsigned long stableStartTime = 0;
    static bool stableAltitudeDetected = false;

    float currentAltitude = baroManager->getAltitude();

    // First time initialization
    if (landingAltitude == 0.0f) {
        landingAltitude = currentAltitude;
        return false;
    }

    // Check if the altitude is stable
    if (abs(currentAltitude - landingAltitude) < LANDED_ALTITUDE_THRESHOLD) {
        if (!stableAltitudeDetected) {
            stableAltitudeDetected = true;
            stableStartTime = millis();
        }

        // Check if we've been stable for long enough
        if (millis() - stableStartTime >= LANDED_STABILITY_TIME) {
            return true;  // Landing confirmed
        }
    } else {
        // Update landing altitude and reset stable detection
        landingAltitude = currentAltitude;
        stableAltitudeDetected = false;
    }

    return false;
}