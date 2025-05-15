/**
 * State Machine - Core Implementation
 */

#include "StateMachine.h"
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"

StateMachine::StateMachine()
        : currentState(RocketState::INIT),
          currentSubState(nullptr),
          barometricManager(nullptr),
          imuManager(nullptr),
          gpsManager(nullptr),
          loraSystem(nullptr),
          storageManager(nullptr),
          lastStateChangeTime(0),
          stateEntryTime(0) {

    // Initialize state handlers to empty functions
    for (int i = 0; i < 10; i++) {
        stateHandlers[i] = []() {};
        eventHandlers[i] = [](RocketEvent) { return false; };
    }
}

void StateMachine::begin(
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        LoRaSystem* loraSystem,
        StorageManager* storageManager,
        DataIntegrationManager* dataManager
) {
    // Store subsystem references
    this->barometricManager = baroManager;
    this->imuManager = imuManager;
    this->gpsManager = gpsManager;
    this->loraSystem = loraSystem;
    this->storageManager = storageManager;
    this->dataManager = dataManager;

    // Initialize state transition table
    initializeTransitions();

    // Set initial substate
    InitSubState initialSubState = InitSubState::HARDWARE_INIT;
    currentSubState = new InitSubState(initialSubState);

    // Set initial state entry time
    stateEntryTime = millis();
    lastStateChangeTime = stateEntryTime;

    // Log initial state
    if (storageManager) {
        storageManager->logMessage(
                LogLevel::INFO,
                Subsystem::STATE_MACHINE,
                "State machine initialized in INIT/HARDWARE_INIT state"
        );
    }
}

void StateMachine::update() {
    // Execute the current state handler
    int stateIndex = static_cast<int>(currentState);
    if (stateIndex >= 0 && stateIndex < 10) {
        stateHandlers[stateIndex]();
    }

    // Update substate mappings
    updateSubState();

    // Update subsystems state tracking
    if (storageManager) {
        storageManager->setSystemState(currentState);
    }
}

bool StateMachine::processEvent(RocketEvent event) {
    // First, let the current state's event handler try to handle it
//    int stateIndex = static_cast<int>(currentState);
//    if (stateIndex >= 0 && stateIndex < 10 && eventHandlers[stateIndex](event)) {
//        Serial.printf("Event %d handled by state %d\n", static_cast<int>(event), stateIndex);
//        return true;  // Event was handled by the state handler
//    }

    Serial.printf("Processing event %d in state %d\n", static_cast<int>(event), static_cast<int>(currentState));

    //Log the event
    if (storageManager) {
        char message[50];
        snprintf(message, sizeof(message), "Event %d in state %d", static_cast<int>(event), static_cast<int>(currentState));
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::STATE_MACHINE, message);
    }

    // Otherwise, check the transition table
    for (const auto& transition : transitions) {
        if (transition.fromState == currentState && transition.event == event) {
            Serial.printf("Transitioning from %d to %d on event %d\n",
                   static_cast<int>(currentState), static_cast<int>(transition.toState), static_cast<int>(event));

            RocketState oldState = currentState;
            changeState(transition.toState);

//            Serial.printf("State changed: %d -> %d\n", static_cast<int>(oldState), static_cast<int>(currentState));

            if (dataManager) {
                dataManager->setCurrentState(currentState);
            }

            // Log the state change
            logStateChange(oldState, currentState);

//            Serial.printf("State changed: %d -> %d\n", static_cast<int>(oldState), static_cast<int>(currentState));

            //Flush the storage manager
            if (storageManager) {
                storageManager->flush();
            }

            return true;
        }
    }

    // If we get here, the event wasn't handled
    if (storageManager) {
        char message[50];
        snprintf(message, sizeof(message), "Unhandled event %d in state %d",
                 static_cast<int>(event), static_cast<int>(currentState));
        storageManager->logMessage(LogLevel::WARNING, Subsystem::STATE_MACHINE, message);

        Serial.printf("Unhandled event %d in state %d\n", static_cast<int>(event), static_cast<int>(currentState));
    }

    return false;
}

RocketState StateMachine::getCurrentState() const {
    return currentState;
}

void* StateMachine::getCurrentSubState() const {
    return currentSubState;
}

bool StateMachine::isInState(RocketState state) const {
    return currentState == state;
}

void StateMachine::registerStateHandler(RocketState state, StateHandler handler) {
    int stateIndex = static_cast<int>(state);
    if (stateIndex >= 0 && stateIndex < 10) {
        stateHandlers[stateIndex] = handler;
    }
}

void StateMachine::registerEventHandler(RocketState state, EventHandler handler) {
    int stateIndex = static_cast<int>(state);
    if (stateIndex >= 0 && stateIndex < 10) {
        eventHandlers[stateIndex] = handler;
    }
}

void StateMachine::initializeTransitions() {
    // Initialize state transition table
    transitions = {
            // INIT state transitions
            {RocketState::INIT, RocketEvent::BOOT_COMPLETED, RocketState::GROUND_IDLE},

            // GROUND_IDLE state transitions
            {RocketState::GROUND_IDLE, RocketEvent::WAKE_UP_COMMAND, RocketState::READY},

            // READY state transitions
            {RocketState::READY, RocketEvent::LAUNCH_COMMAND, RocketState::POWERED_FLIGHT},
            {RocketState::READY, RocketEvent::ACCELERATION_DETECTED, RocketState::POWERED_FLIGHT},

            // POWERED_FLIGHT state transitions
            {RocketState::POWERED_FLIGHT, RocketEvent::ENGINE_BURNOUT, RocketState::COASTING},

            // COASTING state transitions
            {RocketState::COASTING, RocketEvent::APOGEE_DETECTED, RocketState::APOGEE},

            // APOGEE state transitions (immediately goes to DESCENT)
            {RocketState::APOGEE, RocketEvent::APOGEE_DETECTED, RocketState::DESCENT},

            // DESCENT state transitions
            {RocketState::DESCENT, RocketEvent::PARACHUTE_DEPLOYED, RocketState::PARACHUTE_DESCENT},

            // PARACHUTE_DESCENT state transitions
            {RocketState::PARACHUTE_DESCENT, RocketEvent::LANDING_DETECTED, RocketState::LANDED},

            // Error handling transitions
            {RocketState::INIT, RocketEvent::ERROR_DETECTED, RocketState::ERROR},
            {RocketState::GROUND_IDLE, RocketEvent::ERROR_DETECTED, RocketState::ERROR},
            {RocketState::READY, RocketEvent::ERROR_DETECTED, RocketState::ERROR},
            {RocketState::POWERED_FLIGHT, RocketEvent::ERROR_DETECTED, RocketState::ERROR},
            {RocketState::COASTING, RocketEvent::ERROR_DETECTED, RocketState::ERROR},
            {RocketState::APOGEE, RocketEvent::ERROR_DETECTED, RocketState::ERROR},

            // Recovery from error
            {RocketState::ERROR, RocketEvent::RECOVERY_SUCCEEDED, RocketState::INIT},

            // Abort handling
            {RocketState::READY, RocketEvent::ABORT_COMMAND, RocketState::GROUND_IDLE},
//            {RocketState::POWERED_FLIGHT, RocketEvent::ABORT_COMMAND, RocketState::PARACHUTE_DESCENT},
//            {RocketState::COASTING, RocketEvent::ABORT_COMMAND, RocketState::PARACHUTE_DESCENT},
    };

    // Initialize sub-state mappings
    InitSubState initSubState = InitSubState::HARDWARE_INIT;
    GroundIdleSubState idleSubState = GroundIdleSubState::SLEEP;
    ReadySubState readySubState = ReadySubState::ARMED;
    ErrorSubState errorSubState = ErrorSubState::SENSOR_ERROR;

    subStateMappings = {
            {RocketState::INIT, new InitSubState(initSubState)},
            {RocketState::GROUND_IDLE, new GroundIdleSubState(idleSubState)},
            {RocketState::READY, new ReadySubState(readySubState)},
            {RocketState::ERROR, new ErrorSubState(errorSubState)}
    };
}

void StateMachine::updateSubState() {
    // Find mapping for current state
    for (const auto& mapping : subStateMappings) {
        if (mapping.mainState == currentState) {
            // Copy the substate only if it's different type
            if (currentSubState == nullptr || mapping.subState != currentSubState) {
                // Cleanup old substate if needed
                if (currentSubState != nullptr) {
                    switch (currentState) {
                        case RocketState::INIT:
                            delete static_cast<InitSubState*>(currentSubState);
                            break;
                        case RocketState::GROUND_IDLE:
                            delete static_cast<GroundIdleSubState*>(currentSubState);
                            break;
                        case RocketState::READY:
                            delete static_cast<ReadySubState*>(currentSubState);
                            break;
                        case RocketState::ERROR:
                            delete static_cast<ErrorSubState*>(currentSubState);
                            break;
                        default:
                            // No substate to clean
                            break;
                    }
                }

                // Assign new substate
                currentSubState = mapping.subState;

                Serial.printf("Substate changed to %d\n", static_cast<int>(currentState));
                // Log the substate change
                if (storageManager) {
                    char message[50];
                    snprintf(message, sizeof(message), "Substate changed: %d",
                             static_cast<int>(currentState));
                    storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, message);
                }
            }
            break;
        }
    }
}

void StateMachine::changeState(RocketState newState) {
    if (newState == currentState) {
        return;  // No change needed
    }

    Serial.println("Changing state from " + String(static_cast<int>(currentState)) +
                   " to " + String(static_cast<int>(newState)));

    RocketState oldState = currentState;
    currentState = newState;

    // Reset state entry time
    lastStateChangeTime = millis();
    stateEntryTime = lastStateChangeTime;

    Serial.printf("State entry time updated to %lu\n", stateEntryTime);

    // Update substate mapping
    updateSubState();

    // Log state change
    logStateChange(oldState, newState);
}

void StateMachine::logStateChange(RocketState oldState, RocketState newState) {
    if (storageManager) {
        char message[50];
        snprintf(message, sizeof(message), "State changed: %d -> %d",
                 static_cast<int>(oldState), static_cast<int>(newState));
        storageManager->logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, message);
    }
}