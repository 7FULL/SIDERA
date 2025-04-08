#include "CommandHandler.h"

CommandHandler::CommandHandler(
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        StateMachine* stateMachine,
        PowerManager* powerManager,
        DiagnosticManager* diagnosticManager,
        SensorFusionSystem* fusionSystem
)
        : loraSystem(loraSystem),
          storageManager(storageManager),
          stateMachine(stateMachine),
          powerManager(powerManager),
          diagnosticManager(diagnosticManager),
          fusionSystem(fusionSystem),
          telemetryRateMs(1000),  // Default to 1 second
          lastTelemetryTime(0)
{
}

bool CommandHandler::begin() {
    if (!loraSystem) {
        return false;
    }

    // Initialize protocol
    RocketProtocol::initialize();

    // Log initialization
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Command handler initialized");
    }

    lastTelemetryTime = millis();
    return true;
}

void CommandHandler::update() {
    // Process any pending commands
    if (loraSystem && loraSystem->hasReceivedMessages()) {
        Message message;
        while (loraSystem->getNextMessage(message)) {
            // Try to parse as protocol packet
            ProtocolPacket packet;
            if (RocketProtocol::parsePacket(message.data, message.length, packet)) {
                // Handle command based on type
                bool success = false;

                switch (static_cast<CommandCode>(packet.type)) {
                    case CommandCode::PING:
                        success = handlePingCommand(packet);
                        break;
                    case CommandCode::GET_STATUS:
                        success = handleGetStatusCommand(packet);
                        break;
                    case CommandCode::GET_TELEMETRY:
                        success = handleGetTelemetryCommand(packet);
                        break;
                    case CommandCode::ARM_ROCKET:
                        success = handleArmCommand(packet);
                        break;
                    case CommandCode::DISARM_ROCKET:
                        success = handleDisarmCommand(packet);
                        break;
                    case CommandCode::START_COUNTDOWN:
                        success = handleStartCountdownCommand(packet);
                        break;
                    case CommandCode::ABORT_COUNTDOWN:
                        success = handleAbortCountdownCommand(packet);
                        break;
                    case CommandCode::FORCE_DEPLOY_PARACHUTE:
                        success = handleForceDeployParachuteCommand(packet);
                        break;
                    case CommandCode::CALIBRATE_SENSORS:
                        success = handleCalibrateSensorsCommand(packet);
                        break;
                    case CommandCode::RUN_DIAGNOSTICS:
                        success = handleRunDiagnosticsCommand(packet);
                        break;
                    case CommandCode::SET_PARAMETER:
                        success = handleSetParameterCommand(packet);
                        break;
                    case CommandCode::GET_PARAMETER:
                        success = handleGetParameterCommand(packet);
                        break;
                    case CommandCode::ENTER_LOW_POWER:
                        success = handleEnterLowPowerCommand(packet);
                        break;
                    case CommandCode::EXIT_LOW_POWER:
                        success = handleExitLowPowerCommand(packet);
                        break;
                    case CommandCode::RESET_SYSTEM:
                        success = handleResetSystemCommand(packet);
                        break;
                    default:
                        // Unknown command
                        if (storageManager) {
                            char msg[50];
                            snprintf(msg, sizeof(msg), "Unknown command received: 0x%02X", packet.type);
                            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION, msg);
                        }

                        // Send NACK
                        sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
                        break;
                }

                // Clean up packet payload if needed
                if (packet.payload) {
                    delete[] packet.payload;
                }
            } else {
                // Invalid or corrupt packet
                if (storageManager) {
                    storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                               "Received invalid packet");
                }
            }
        }
    }

    // Send periodic telemetry if needed
    unsigned long currentTime = millis();
    if (currentTime - lastTelemetryTime >= telemetryRateMs) {
        lastTelemetryTime = currentTime;
        sendTelemetry();
    }
}

bool CommandHandler::sendResponse(ResponseCode code, const uint8_t* payload,
                                  uint16_t length, uint16_t sequenceNumber) {
    if (!loraSystem) {
        return false;
    }

    // Create response packet
    std::vector<uint8_t> packetData =
            RocketProtocol::createResponsePacket(code, payload, length, sequenceNumber);

    // Create LoRa message
    Message message;
    message.type = MessageType::COMMAND_RESPONSE;
    message.priority = 200;  // High priority for responses
    message.timestamp = millis();
    message.length = packetData.size();
    message.data = new uint8_t[message.length];
    memcpy(message.data, packetData.data(), message.length);
    message.acknowledged = false;

    // Send the message
    bool result = loraSystem->sendMessage(message);

    // Clean up data
    delete[] message.data;

    return result;
}

bool CommandHandler::sendTelemetry() {
    if (!loraSystem || !fusionSystem) {
        return false;
    }

    // Create telemetry payload
    std::vector<uint8_t> payload = createTelemetryPayload();

    // Send telemetry response
    return sendResponse(ResponseCode::TELEMETRY_DATA, payload.data(), payload.size());
}

bool CommandHandler::sendErrorMessage(const char* message) {
    if (!loraSystem || !message) {
        return false;
    }

    // Create error payload
    size_t msgLen = strlen(message);
    if (msgLen > 100) msgLen = 100;  // Limit message length

    // Send error response
    return sendResponse(ResponseCode::ERROR_MESSAGE,
                        reinterpret_cast<const uint8_t*>(message), msgLen);
}

bool CommandHandler::sendEventNotification(uint8_t eventCode, const char* description) {
    if (!loraSystem || !description) {
        return false;
    }

    // Create event payload
    size_t descLen = strlen(description);
    if (descLen > 99) descLen = 99;  // Limit description length

    uint8_t payload[100];
    payload[0] = eventCode;  // First byte is event code
    memcpy(&payload[1], description, descLen);  // Rest is description

    // Send event notification
    return sendResponse(ResponseCode::EVENT_NOTIFICATION, payload, descLen + 1);
}

void CommandHandler::setTelemetryRate(uint16_t rateMs) {
    telemetryRateMs = rateMs;

    // Log rate change
    if (storageManager) {
        char msg[50];
        snprintf(msg, sizeof(msg), "Telemetry rate set to %d ms", rateMs);
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION, msg);
    }
}

bool CommandHandler::handlePingCommand(const ProtocolPacket& packet) {
    // Log ping
    if (storageManager) {
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::COMMUNICATION, "Ping received");
    }

    // Send ACK with timestamp
    uint32_t timestamp = millis();
    uint8_t payload[4];
    payload[0] = (timestamp >> 24) & 0xFF;
    payload[1] = (timestamp >> 16) & 0xFF;
    payload[2] = (timestamp >> 8) & 0xFF;
    payload[3] = timestamp & 0xFF;

    return sendResponse(ResponseCode::ACK, payload, 4, packet.sequenceNumber);
}

bool CommandHandler::handleGetStatusCommand(const ProtocolPacket& packet) {
    if (!stateMachine) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Get current state
    RocketState currentState = stateMachine->getCurrentState();

    // Map to status code
    RocketStatusCode statusCode;
    switch (currentState) {
        case RocketState::INIT:
            statusCode = RocketStatusCode::INITIALIZING;
            break;
        case RocketState::GROUND_IDLE:
            statusCode = RocketStatusCode::IDLE;
            break;
        case RocketState::READY:
            // Check substate
            if (stateMachine->isInSubState(ReadySubState::ARMED)) {
                statusCode = RocketStatusCode::ARMED;
            } else if (stateMachine->isInSubState(ReadySubState::COUNTDOWN)) {
                statusCode = RocketStatusCode::COUNTDOWN;
            } else {
                statusCode = RocketStatusCode::READY;
            }
            break;
        case RocketState::POWERED_FLIGHT:
            statusCode = RocketStatusCode::POWERED_FLIGHT;
            break;
        case RocketState::COASTING:
            statusCode = RocketStatusCode::COASTING;
            break;
        case RocketState::APOGEE:
            statusCode = RocketStatusCode::APOGEE;
            break;
        case RocketState::DESCENT:
            statusCode = RocketStatusCode::DESCENT;
            break;
        case RocketState::PARACHUTE_DESCENT:
            statusCode = RocketStatusCode::PARACHUTE_DEPLOYED;
            break;
        case RocketState::LANDED:
            statusCode = RocketStatusCode::LANDED;
            break;
        case RocketState::ERROR:
            statusCode = RocketStatusCode::ERROR;
            break;
        default:
            statusCode = RocketStatusCode::ERROR;
            break;
    }

    // Create status payload
    uint8_t payload[18];
    payload[0] = static_cast<uint8_t>(statusCode);

    // Add battery info
    float voltage = 0.0f;
    int percentage = 0;
    if (powerManager) {
        voltage = powerManager->getBatteryVoltage();
        percentage = powerManager->getBatteryPercentage();
    }

    uint16_t voltageInt = static_cast<uint16_t>(voltage * 100);  // Convert to centivolts
    payload[1] = (voltageInt >> 8) & 0xFF;
    payload[2] = voltageInt & 0xFF;
    payload[3] = percentage;

    // Add system uptime
    uint32_t uptime = millis() / 1000;  // In seconds
    payload[4] = (uptime >> 24) & 0xFF;
    payload[5] = (uptime >> 16) & 0xFF;
    payload[6] = (uptime >> 8) & 0xFF;
    payload[7] = uptime & 0xFF;

    // Add free storage space
    uint32_t freeSpace = 0;
    if (storageManager) {
        freeSpace = storageManager->getAvailableSpace();
    }
    payload[8] = (freeSpace >> 24) & 0xFF;
    payload[9] = (freeSpace >> 16) & 0xFF;
    payload[10] = (freeSpace >> 8) & 0xFF;
    payload[11] = freeSpace & 0xFF;

    // Add GPS fix status
    bool gpsFix = false;
    if (fusionSystem) {
        FusedFlightData data = fusionSystem->getFusedData();
        // Add some GPS status flags here
    }
    payload[12] = gpsFix ? 1 : 0;

    // Add sensor status flags
    uint8_t sensorFlags = 0;
    // Each bit represents a sensor system
    // Set appropriate bits based on sensor status
    payload[13] = sensorFlags;

    // Add error flags
    uint32_t errorFlags = 0;
    payload[14] = (errorFlags >> 24) & 0xFF;
    payload[15] = (errorFlags >> 16) & 0xFF;
    payload[16] = (errorFlags >> 8) & 0xFF;
    payload[17] = errorFlags & 0xFF;

    return sendResponse(ResponseCode::STATUS_DATA, payload, sizeof(payload), packet.sequenceNumber);
}

bool CommandHandler::handleGetTelemetryCommand(const ProtocolPacket& packet) {
    // Simply trigger a telemetry send
    return sendTelemetry();
}

bool CommandHandler::handleArmCommand(const ProtocolPacket& packet) {
    if (!stateMachine) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Check if in appropriate state
    if (stateMachine->getCurrentState() != RocketState::READY) {
        // Can only arm in READY state
        if (storageManager) {
            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                       "Arm command rejected - not in READY state");
        }
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Process ARM event
    stateMachine->processEvent(RocketEvent::ARM_COMMAND);

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Rocket armed via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleDisarmCommand(const ProtocolPacket& packet) {
    if (!stateMachine) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Check if in appropriate state
    if (stateMachine->getCurrentState() != RocketState::READY) {
        // Can only disarm in READY state
        if (storageManager) {
            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                       "Disarm command rejected - not in READY state");
        }
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Process ARM event (should switch to disarmed state)
    // You may need to define a DISARM_COMMAND event
    stateMachine->processEvent(RocketEvent::ABORT_COMMAND);

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Rocket disarmed via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleStartCountdownCommand(const ProtocolPacket& packet) {
    if (!stateMachine) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Check if in appropriate state
    if (stateMachine->getCurrentState() != RocketState::READY) {
        // Can only start countdown in READY state
        if (storageManager) {
            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                       "Countdown command rejected - not in READY state");
        }
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Process LAUNCH event
    stateMachine->processEvent(RocketEvent::LAUNCH_COMMAND);

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Countdown started via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleAbortCountdownCommand(const ProtocolPacket& packet) {
    if (!stateMachine) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Process ABORT event
    stateMachine->processEvent(RocketEvent::ABORT_COMMAND);

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Countdown aborted via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleForceDeployParachuteCommand(const ProtocolPacket& packet) {
    if (!stateMachine) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Only allow if in flight phases
    RocketState currentState = stateMachine->getCurrentState();
    if (currentState != RocketState::POWERED_FLIGHT &&
        currentState != RocketState::COASTING &&
        currentState != RocketState::APOGEE &&
        currentState != RocketState::DESCENT) {

        if (storageManager) {
            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                       "Deploy parachute command rejected - not in flight");
        }
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Force transition to parachute descent
    stateMachine->processEvent(RocketEvent::PARACHUTE_DEPLOYED);

    if (storageManager) {
        storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                   "Parachute manually deployed via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleCalibrateSensorsCommand(const ProtocolPacket& packet) {
    // Only allow in GROUND_IDLE or READY states
    if (stateMachine) {
        RocketState currentState = stateMachine->getCurrentState();
        if (currentState != RocketState::GROUND_IDLE &&
            currentState != RocketState::READY) {

            if (storageManager) {
                storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                           "Calibration rejected - not in appropriate state");
            }
            return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
        }
    }

    // Implement sensor calibration here
    // This would call appropriate calibration methods on your sensor systems

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Sensor calibration initiated via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleRunDiagnosticsCommand(const ProtocolPacket& packet) {
    if (!diagnosticManager) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Run diagnostics
    std::vector<TestResult> results = diagnosticManager->runAllTests();

    // Prepare response - first byte is test count, then each test result
    std::vector<uint8_t> payload;
    payload.push_back(results.size());

    for (const auto& result : results) {
        // Add pass/fail status (1 byte)
        payload.push_back(result.passed ? 1 : 0);

        // Add test name (up to 16 bytes)
        String testName = result.testName;
        if (testName.length() > 16) testName = testName.substring(0, 16);
        payload.push_back(testName.length());
        for (size_t i = 0; i < testName.length(); i++) {
            payload.push_back(testName[i]);
        }

        // Add error message for failed tests (up to 32 bytes)
        if (!result.passed) {
            String errorMsg = result.errorMessage;
            if (errorMsg.length() > 32) errorMsg = errorMsg.substring(0, 32);
            payload.push_back(errorMsg.length());
            for (size_t i = 0; i < errorMsg.length(); i++) {
                payload.push_back(errorMsg[i]);
            }
        } else {
            payload.push_back(0);  // No error message
        }
    }

    return sendResponse(ResponseCode::DIAGNOSTIC_RESULT, payload.data(), payload.size(),
                        packet.sequenceNumber);
}

bool CommandHandler::handleSetParameterCommand(const ProtocolPacket& packet) {
    if (!packet.payload || packet.length < 3) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Extract parameter ID and value
    uint8_t paramId = packet.payload[0];
    uint16_t paramValue = (packet.payload[1] << 8) | packet.payload[2];

    // Handle different parameters
    switch (paramId) {
        case ParameterId::TELEMETRY_RATE:
            setTelemetryRate(paramValue);
            break;

        case ParameterId::SENSOR_UPDATE_RATE:
            // Set sensor update rate
            break;

        case ParameterId::LORA_POWER:
            if (loraSystem) {
                loraSystem->setTxPower(paramValue);
            }
            break;

        case ParameterId::APOGEE_DETECTION_THRESHOLD:
            // Set apogee detection threshold
            break;

        case ParameterId::LANDING_DETECTION_THRESHOLD:
            // Set landing detection threshold
            break;

        case ParameterId::LOGGING_LEVEL:
            // Set logging level
            break;

        default:
            // Unknown parameter
            return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    if (storageManager) {
        char msg[50];
        snprintf(msg, sizeof(msg), "Parameter %d set to %d", paramId, paramValue);
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION, msg);
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleGetParameterCommand(const ProtocolPacket& packet) {
    if (!packet.payload || packet.length < 1) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Extract parameter ID
    uint8_t paramId = packet.payload[0];
    uint16_t paramValue = 0;

    // Get parameter value
    switch (paramId) {
        case ParameterId::TELEMETRY_RATE:
            paramValue = telemetryRateMs;
            break;

        case ParameterId::SENSOR_UPDATE_RATE:
            // Get sensor update rate
            break;

        case ParameterId::LORA_POWER:
            // Get LoRa TX power
            break;

        case ParameterId::APOGEE_DETECTION_THRESHOLD:
            // Get apogee detection threshold
            break;

        case ParameterId::LANDING_DETECTION_THRESHOLD:
            // Get landing detection threshold
            break;

        case ParameterId::LOGGING_LEVEL:
            // Get logging level
            break;

        default:
            // Unknown parameter
            return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Prepare response
    uint8_t payload[3];
    payload[0] = paramId;
    payload[1] = (paramValue >> 8) & 0xFF;
    payload[2] = paramValue & 0xFF;

    return sendResponse(ResponseCode::PARAMETER_VALUE, payload, 3, packet.sequenceNumber);
}

bool CommandHandler::handleEnterLowPowerCommand(const ProtocolPacket& packet) {
    if (!powerManager) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Enter low power mode
    powerManager->enterPowerState(PowerState::LOW_POWER);

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Entering low power mode via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleExitLowPowerCommand(const ProtocolPacket& packet) {
    if (!powerManager) {
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Exit low power mode
    powerManager->enterPowerState(PowerState::NORMAL);

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Exiting low power mode via command");
    }

    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleResetSystemCommand(const ProtocolPacket& packet) {
    if (storageManager) {
        storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                   "System reset commanded");
    }

    // Send ACK before reset
    sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);

    // Wait for message to be sent
    delay(500);

    // Reset processor
    //TODO
//    NVIC_SystemReset();

    return true;
}

std::vector<uint8_t> CommandHandler::createTelemetryPayload() {
    std::vector<uint8_t> payload;

    // Add timestamp (4 bytes)
    uint32_t timestamp = millis();
    payload.push_back((timestamp >> 24) & 0xFF);
    payload.push_back((timestamp >> 16) & 0xFF);
    payload.push_back((timestamp >> 8) & 0xFF);
    payload.push_back(timestamp & 0xFF);

    // Add rocket state (1 byte)
    uint8_t state = 0;
    if (stateMachine) {
        state = static_cast<uint8_t>(stateMachine->getCurrentState());
    }
    payload.push_back(state);

    // Add sensor fusion data if available
    if (fusionSystem) {
        FusedFlightData data = fusionSystem->getFusedData();

        // Altitude (4 bytes, float as int32)
        int32_t altitude = static_cast<int32_t>(data.altitude * 1000);  // mm
        payload.push_back((altitude >> 24) & 0xFF);
        payload.push_back((altitude >> 16) & 0xFF);
        payload.push_back((altitude >> 8) & 0xFF);
        payload.push_back(altitude & 0xFF);

        // Vertical speed (2 bytes, scaled int16)
        int16_t vertSpeed = static_cast<int16_t>(data.verticalSpeed * 100);  // cm/s
        payload.push_back((vertSpeed >> 8) & 0xFF);
        payload.push_back(vertSpeed & 0xFF);

        // Vertical acceleration (2 bytes, scaled int16)
        int16_t vertAccel = static_cast<int16_t>(data.verticalAccel * 100);  // cm/s²
        payload.push_back((vertAccel >> 8) & 0xFF);
        payload.push_back(vertAccel & 0xFF);

        // Orientation (3x2 bytes)
        int16_t roll = static_cast<int16_t>(data.roll * 10);  // 0.1 degrees
        int16_t pitch = static_cast<int16_t>(data.pitch * 10);
        int16_t yaw = static_cast<int16_t>(data.yaw * 10);

        payload.push_back((roll >> 8) & 0xFF);
        payload.push_back(roll & 0xFF);
        payload.push_back((pitch >> 8) & 0xFF);
        payload.push_back(pitch & 0xFF);
        payload.push_back((yaw >> 8) & 0xFF);
        payload.push_back(yaw & 0xFF);

        // Confidence (1 byte, scaled 0-255)
        uint8_t confidence = static_cast<uint8_t>(data.confidence * 255);
        payload.push_back(confidence);
    } else {
        // Add zeros if no fusion data
        for (int i = 0; i < 14; i++) {
            payload.push_back(0);
        }
    }

    // Add battery info (3 bytes)
    if (powerManager) {
        float voltage = powerManager->getBatteryVoltage();
        int percentage = powerManager->getBatteryPercentage();

        uint16_t voltageInt = static_cast<uint16_t>(voltage * 100);  // centivolts
        payload.push_back((voltageInt >> 8) & 0xFF);
        payload.push_back(voltageInt & 0xFF);
        payload.push_back(percentage);
    } else {
        payload.push_back(0);
        payload.push_back(0);
        payload.push_back(0);
    }

    // Add GPS info if available (10 bytes)
    // Could be expanded with more detailed GPS data

    // Add temperature (2 bytes, scaled int16)
    int16_t temperature = 0;  // 0.1 C
    payload.push_back((temperature >> 8) & 0xFF);
    payload.push_back(temperature & 0xFF);

    // Add pressure (3 bytes)
    uint32_t pressure = 0;  // Pa
    payload.push_back((pressure >> 16) & 0xFF);
    payload.push_back((pressure >> 8) & 0xFF);
    payload.push_back(pressure & 0xFF);

    // Add status flags (1 byte)
    uint8_t flags = 0;
    payload.push_back(flags);

    return payload;
}