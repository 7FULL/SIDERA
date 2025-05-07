/**
 * State Handlers - Implementation
 *
 * Specific handler implementations for each rocket state
 */

#ifndef STATE_HANDLERS_H
#define STATE_HANDLERS_H

#include "StateMachine.h"
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"
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
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataManager,
            DiagnosticManager* diagnosticManager,
            PreflightCheckSystem* preflightSystem,
            PowerManager* powerManager
    );

    static void sendTelemetryData(
            LoRaSystem* loraSystem,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            DataIntegrationManager* dataIntegrationManager,
            PowerManager* powerManager,
            uint8_t rocketState
    );

    // Individual state handlers
    static void handleInitState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            DiagnosticManager* diagnosticManager,
            PreflightCheckSystem* preflightSystem
    );

    static void handleGroundIdleState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            PowerManager* powerManager
    );

    static void handleReadyState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            DiagnosticManager* diagnosticManager,
            PowerManager* powerManager
    );

    static void handlePoweredFlightState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            PowerManager* powerManager
    );

    static void handleCoastingState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            PowerManager* powerManager
    );

    static void handleApogeeState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager
    );

    static void handleDescentState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager
    );

    static void handleParachuteDescentState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            PowerManager* powerManager
    );

    static void handleLandedState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager,
            PowerManager* powerManager
    );

    static void handleErrorState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager
    );

    static bool handleGroundIdleEvents(
            RocketEvent event,
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            DataIntegrationManager* dataIntegrationManager
    );

    // Helpers
    static void deployParachute();
    static bool detectLaunch(IMUSensorManager* imuManager, DataIntegrationManager* dataManager);
    static bool detectApogee(BarometricSensorManager* baroManager, DataIntegrationManager* dataManager);
    static bool detectLanding(BarometricSensorManager* baroManager, DataIntegrationManager* dataManager);

    // Timekeeping
    static unsigned long lastSensorUpdateTime;
    static unsigned long lastTelemetryTime;
    static unsigned long flightStartTime;
};

#endif // STATE_HANDLERS_H