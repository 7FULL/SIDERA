#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "RocketProtocol.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include "../StateMachine/StateMachine.h"
#include "../PowerManagement/PowerManager.h"
#include "../Diagnostics/DiagnosticManager.h"
#include "../SensorFusion/SensorFusionSystem.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"

class CommandHandler {
public:
    CommandHandler(
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            StateMachine* stateMachine,
            PowerManager* powerManager,
            DiagnosticManager* diagnosticManager,
            SensorFusionSystem* fusionSystem,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager
    );

    // Initialize command handler
    bool begin();

    // Process incoming messages
    void update();

    // Send response
    bool sendResponse(ResponseCode code, const uint8_t* payload = nullptr,
                      uint16_t length = 0, uint16_t sequenceNumber = 0);

    // Send error message
    bool sendErrorMessage(const char* message);

    // Send event notification
    bool sendEventNotification(uint8_t eventCode, const char* description);

private:
    LoRaSystem* loraSystem;
    StorageManager* storageManager;
    StateMachine* stateMachine;
    PowerManager* powerManager;
    DiagnosticManager* diagnosticManager;
    SensorFusionSystem* fusionSystem;
    BarometricSensorManager* baroManager;
    IMUSensorManager* imuManager;
    GPSSensorManager* gpsManager;

    // Handle specific commands
    bool handlePingCommand(const ProtocolPacket& packet);
    bool handleGetStatusCommand(const ProtocolPacket& packet);
    bool handleWakeUpCommand(const ProtocolPacket& packet);
    bool handleCalibrateSensorsCommand(const ProtocolPacket& packet);
    bool handleRunDiagnosticsCommand(const ProtocolPacket& packet);
    bool handleAbortCommand(const ProtocolPacket& packet);
};

#endif // COMMAND_HANDLER_H