#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "RocketProtocol.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include "../StateMachine/StateMachine.h"
#include "../PowerManagement/PowerManager.h"
#include "../Diagnostics/DiagnosticManager.h"
#include "../SensorFusion/SensorFusionSystem.h"

class CommandHandler {
public:
    CommandHandler(
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            StateMachine* stateMachine,
            PowerManager* powerManager,
            DiagnosticManager* diagnosticManager,
            SensorFusionSystem* fusionSystem
    );

    // Initialize command handler
    bool begin();

    // Process incoming messages
    void update();

    // Send response
    bool sendResponse(ResponseCode code, const uint8_t* payload = nullptr,
                      uint16_t length = 0, uint16_t sequenceNumber = 0);

    // Send telemetry data
    bool sendTelemetry();

    // Send error message
    bool sendErrorMessage(const char* message);

    // Send event notification
    bool sendEventNotification(uint8_t eventCode, const char* description);

    // Set telemetry rate
    void setTelemetryRate(uint16_t rateMs);

private:
    LoRaSystem* loraSystem;
    StorageManager* storageManager;
    StateMachine* stateMachine;
    PowerManager* powerManager;
    DiagnosticManager* diagnosticManager;
    SensorFusionSystem* fusionSystem;

    // Telemetry timing
    uint16_t telemetryRateMs;
    unsigned long lastTelemetryTime;

    // Handle specific commands
    bool handlePingCommand(const ProtocolPacket& packet);
    bool handleGetStatusCommand(const ProtocolPacket& packet);
    bool handleGetTelemetryCommand(const ProtocolPacket& packet);
    bool handleArmCommand(const ProtocolPacket& packet);
    bool handleDisarmCommand(const ProtocolPacket& packet);
    bool handleStartCountdownCommand(const ProtocolPacket& packet);
    bool handleAbortCountdownCommand(const ProtocolPacket& packet);
    bool handleForceDeployParachuteCommand(const ProtocolPacket& packet);
    bool handleCalibrateSensorsCommand(const ProtocolPacket& packet);
    bool handleRunDiagnosticsCommand(const ProtocolPacket& packet);
    bool handleSetParameterCommand(const ProtocolPacket& packet);
    bool handleGetParameterCommand(const ProtocolPacket& packet);
    bool handleEnterLowPowerCommand(const ProtocolPacket& packet);
    bool handleExitLowPowerCommand(const ProtocolPacket& packet);
    bool handleResetSystemCommand(const ProtocolPacket& packet);

    // Create telemetry payload
    std::vector<uint8_t> createTelemetryPayload();
};

#endif // COMMAND_HANDLER_H