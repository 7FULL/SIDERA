/**
 * State Machine - Core Implementation
 *
 * Hierarchical state machine for controlling rocket behavior
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "States.h"
#include "SensorFusionSystem.h"
#include <functional>
#include <vector>
#include <Arduino.h>

// Forward declarations
class BarometricSensorManager;
class IMUSensorManager;
class GPSSensorManager;
class LoRaSystem;
class StorageManager;

// State handler function type
using StateHandler = std::function<void(void)>;

// Event handler type (returns true if event was handled)
using EventHandler = std::function<bool(RocketEvent)>;

// State transition structure
struct StateTransition {
    RocketState fromState;
    RocketEvent event;
    RocketState toState;
};

// Sub-state mapping structure
struct SubStateMapping {
    RocketState mainState;
    void* subState;  // Generic pointer to enum class value
};

class StateMachine {
public:
    StateMachine();

    // Initialize the state machine with required subsystems
    void begin(
            BarometricSensorManager* baroManager,
            IMUSensorManager* imuManager,
            GPSSensorManager* gpsManager,
            LoRaSystem* loraSystem,
            StorageManager* storageManager,
            SensorFusionSystem* fusionSystem = nullptr
    );

    // Update the state machine (called in each loop iteration)
    void update();

    // Process an event
    bool processEvent(RocketEvent event);

    // Get the current main state
    RocketState getCurrentState() const;

    // Get the current sub-state as a void pointer (must be cast by caller)
    void* getCurrentSubState() const;

    // Check if current state is equal to the given state
    bool isInState(RocketState state) const;

    // Check if in a specific substate
    template<typename T>
    bool isInSubState(T subState) const;

    // Register a state handler
    void registerStateHandler(RocketState state, StateHandler handler);

    // Register an event handler for a specific state
    void registerEventHandler(RocketState state, EventHandler handler);

private:
    // Current state
    RocketState currentState;
    void* currentSubState;

    // State handlers
    StateHandler stateHandlers[10];  // One for each RocketState

    // Event handlers
    EventHandler eventHandlers[10];  // One for each RocketState

    // State transitions
    std::vector<StateTransition> transitions;

    // Sub-state mappings
    std::vector<SubStateMapping> subStateMappings;

    // Helper methods
    void initializeTransitions();
    void updateSubState();
    void changeState(RocketState newState);
    void logStateChange(RocketState oldState, RocketState newState);

    // Subsystem references
    BarometricSensorManager* barometricManager;
    IMUSensorManager* imuManager;
    GPSSensorManager* gpsManager;
    LoRaSystem* loraSystem;
    StorageManager* storageManager;
    SensorFusionSystem* fusionSystem;

    // Timekeeping
    unsigned long lastStateChangeTime;
    unsigned long stateEntryTime;
};

// Template implementation
template<typename T>
bool StateMachine::isInSubState(T subState) const {
    if (currentSubState == nullptr) {
        return false;
    }

    // Cast and compare
    return *static_cast<T*>(currentSubState) == subState;
}

#endif // STATE_MACHINE_H