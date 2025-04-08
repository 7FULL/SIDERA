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
#include "../SensorFusion/SensorFusionSystem.h"

class StateHandlers {
public:
    static void setupHandlers(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

private:
    // Individual state handlers
    static void handleInitState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleGroundIdleState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleReadyState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handlePoweredFlightState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleCoastingState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleApogeeState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleDescentState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleParachuteDescentState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleLandedState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static void handleErrorState(
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    // Event handlers
    static bool handleInitEvents(
            RocketEvent event,
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    static bool handleGroundIdleEvents(
            RocketEvent event,
            StateMachine& stateMachine,
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem
    );

    // Helpers
    static void deployParachute();
    static bool detectLaunch(IMUSensorManager* imuManager, SensorFusionSystem* fusionSystem);
    static bool detectApogee(BarometricSensorManager* baroManager, SensorFusionSystem* fusionSystem);
    static bool detectLanding(BarometricSensorManager* baroManager, SensorFusionSystem* fusionSystem);

    // Timekeeping
    static unsigned long lastSensorUpdateTime;
    static unsigned long lastTelemetryTime;
    static unsigned long flightStartTime;
};

#endif // STATE_HANDLERS_H