/**
 * Rocket Control System - State Definitions
 *
 * This file defines the states for the hierarchical state machine.
 */

#ifndef STATES_H
#define STATES_H

// Main states
enum class RocketState {
    INIT,               // Initial state during boot
    GROUND_IDLE,        // On ground, waiting for commands
    READY,              // Ready for launch, all systems go
    POWERED_FLIGHT,     // Engine burning, accelerating
    COASTING,           // Engine off, still ascending
    APOGEE,             // At highest point
    DESCENT,            // Falling, no parachute
    PARACHUTE_DESCENT,  // Falling with parachute deployed
    LANDED,             // On ground after flight
    ERROR               // Error condition
};

// Sub-states
enum class InitSubState {
    HARDWARE_INIT,
    SENSOR_CALIBRATION,
    SELF_TEST
};

enum class GroundIdleSubState {
    SLEEP,
    RECEIVING_COMMANDS,
    LOW_POWER
};

enum class ReadySubState {
    ARMED,
    COUNTDOWN
};

enum class ErrorSubState {
    SENSOR_ERROR,
    COMMUNICATION_ERROR,
    STORAGE_ERROR,
    BATTERY_LOW,
    RECOVERY_MODE
};

// Events that can trigger state transitions
enum class RocketEvent {
    BOOT_COMPLETED,
    CALIBRATION_DONE,
    WAKE_UP_COMMAND,
    ARM_COMMAND,
    LAUNCH_COMMAND,
    ACCELERATION_DETECTED,
    ENGINE_BURNOUT,
    APOGEE_DETECTED,
    PARACHUTE_DEPLOYED,
    LANDING_DETECTED,
    ERROR_DETECTED,
    RECOVERY_MODE_ENTERED,
    RECOVERY_SUCCEEDED,
    ABORT_COMMAND
};

#endif // STATES_H