#include "CommandHandler.h"
#include "../../include/States.h"

CommandHandler::CommandHandler(
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        StateMachine* stateMachine,
        PowerManager* powerManager,
        DiagnosticManager* diagnosticManager,
        SensorFusionSystem* fusionSystem,
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager
)
        :   loraSystem(loraSystem),
            storageManager(storageManager),
            stateMachine(stateMachine),
            powerManager(powerManager),
            diagnosticManager(diagnosticManager),
            fusionSystem(fusionSystem),
            baroManager(baroManager),
            imuManager(imuManager),
            gpsManager(gpsManager)
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
    return true;
}

void CommandHandler::update() {
    // Process any pending commands
    if (loraSystem && loraSystem->hasReceivedMessages()) {
        Message message;
        while (loraSystem->getNextMessage(message)) {
            // Check if this is a simple command (length == 1)
            if (message.length == 1) {
                // This is a simple command from the control panel
                uint8_t commandType = message.data[0];
                Serial.printf("Received simple command: %d\n", commandType);

                // Process simple command directly
                bool success = false;

                switch (static_cast<CommandCode>(commandType)) {
                    case CommandCode::PING:
                        success = handlePingCommand(ProtocolPacket());
                        break;
                    case CommandCode::GET_STATUS:
                        success = handleGetStatusCommand(ProtocolPacket());
                        break;
                    case CommandCode::WAKE_UP_COMMAND:
                        success = handleWakeUpCommand(ProtocolPacket());
                        break;
                    case CommandCode::ABORT_COMMAND:
                        success = handleAbortCommand(ProtocolPacket());
                        break;
                    case CommandCode::CALIBRATE_SENSORS:
                        success = handleCalibrateSensorsCommand(ProtocolPacket());
                        break;
                    case CommandCode::RUN_DIAGNOSTICS:
                        success = handleRunDiagnosticsCommand(ProtocolPacket());
                        break;
                    default:
                        // Unknown command
                        if (storageManager) {
                            char msg[50];
                            snprintf(msg, sizeof(msg), "Unknown simple command received: 0x%02X", commandType);
                            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION, msg);
                        }
                        break;
                }

                // Clean up message data
                if (message.data) {
                    delete[] message.data;
                }
            }
                // Handle other message formats if needed
            else {
                // Clean up message data
                if (message.data) {
                    delete[] message.data;
                }
            }
        }
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

bool CommandHandler::handleWakeUpCommand(const ProtocolPacket& packet) {
    Serial.println("DEBUG: Processing WAKE_UP_COMMAND");

    if (!stateMachine) {
        Serial.println("DEBUG: State machine is NULL!");
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Check current state
    RocketState currentState = stateMachine->getCurrentState();
    Serial.print("DEBUG: Current state is: ");
    Serial.println(static_cast<int>(currentState));

    // Only allow wake-up from GROUND_IDLE state
    if (currentState != RocketState::GROUND_IDLE) {
        Serial.println("DEBUG: Wake-up command rejected - not in GROUND_IDLE state");
        if (storageManager) {
            storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                       "Wake-up command rejected - not in GROUND_IDLE state");
        }
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Process WAKE_UP event
    Serial.println("DEBUG: Sending WAKE_UP_COMMAND event to state machine");
    bool result = stateMachine->processEvent(RocketEvent::WAKE_UP_COMMAND);
    Serial.print("DEBUG: Event processing result: ");
    Serial.println(result ? "SUCCESS" : "FAILED");

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::COMMUNICATION,
                                   "Rocket awakened from idle state via command");
    }

    Serial.println("DEBUG: Sending ACK response");
    return sendResponse(ResponseCode::ACK, nullptr, 0, packet.sequenceNumber);
}

bool CommandHandler::handleAbortCommand(const ProtocolPacket& packet) {
    Serial.println("DEBUG: Processing ABORT_COMMAND");

    if (!stateMachine) {
        Serial.println("DEBUG: State machine is NULL!");
        return sendResponse(ResponseCode::NACK, nullptr, 0, packet.sequenceNumber);
    }

    // Process ABORT event - works in any state
    Serial.println("DEBUG: Sending ABORT_COMMAND event to state machine");
    bool result = stateMachine->processEvent(RocketEvent::ABORT_COMMAND);
    Serial.print("DEBUG: Event processing result: ");
    Serial.println(result ? "SUCCESS" : "FAILED");

    if (storageManager) {
        storageManager->logMessage(LogLevel::WARNING, Subsystem::COMMUNICATION,
                                   "Abort command received");
    }

    Serial.println("DEBUG: Sending ACK response");
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