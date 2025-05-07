/**
 * State Handlers - Implementation
 *
 * Specific handler implementations for each rocket state
 */

#ifndef STATE_HANDLERS_H
#define STATE_HANDLERS_H

#include "StateMachine.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include "../PowerManagement/PowerManager.h"
#include "../PowerManagement/PowerController.h"
#include "../Diagnostics/DiagnosticManager.h"
#include "../Diagnostics/PreflightCheck.h"
#include "../HAL/CommunicationSystems/TelemetrySerializer.h"

class StateHandlers {
public:
    static void setupHandlers(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DiagnosticManager* diagnosticManager,
            PreflightCheckSystem* preflightSystem,
            PowerManager* powerManager
    );

    static void sendTelemetryData(
            LoRaSystem* loraSystem,
            DataIntegrationManager* dataManager,
            PowerManager* powerManager,
            uint8_t rocketState
    );

    // Individual state handlers
    static void handleInitState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DiagnosticManager* diagnosticManager,
            PreflightCheckSystem* preflightSystem
    );

    static void handleGroundIdleState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            PowerManager* powerManager
    );

    static void handleReadyState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DiagnosticManager* diagnosticManager,
            PowerManager* powerManager
    );

    static void handlePoweredFlightState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            PowerManager* powerManager
    );

    static void handleCoastingState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            PowerManager* powerManager
    );

    static void handleApogeeState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager
    );

    static void handleDescentState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager
    );

    static void handleParachuteDescentState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            PowerManager* powerManager
    );

    static void handleLandedState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            PowerManager* powerManager
    );

    static void handleErrorState(
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager
    );

    static bool handleGroundIdleEvents(
            RocketEvent event,
            StateMachine& stateMachine,
            DataIntegrationManager* dataManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager
    );

    // Helper methods
    static void deployParachute();
    static bool detectLaunch(DataIntegrationManager* dataManager);
    static bool detectApogee(DataIntegrationManager* dataManager);
    static bool detectLanding(DataIntegrationManager* dataManager);

    // Timekeeping
    static unsigned long lastSensorUpdateTime;
    static unsigned long lastTelemetryTime;
    static unsigned long flightStartTime;
};

#endif // STATE_HANDLERS_H