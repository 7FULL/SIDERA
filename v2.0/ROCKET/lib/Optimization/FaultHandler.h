#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <Arduino.h>
#include "../HAL/StorageSystems/StorageManager.h"
#include "../StateMachine/StateMachine.h"

// Fault types that can be handled
enum class FaultType {
    MEMORY_ERROR,
    SENSOR_TIMEOUT,
    COMMUNICATION_ERROR,
    STATE_MACHINE_ERROR,
    CRITICAL_TASK_TIMEOUT,
    UNEXPECTED_RESET,
    HARDWARE_FAILURE,
    INTERNAL_ERROR
};

// Fault severity levels
enum class FaultSeverity {
    INFO,        // Not an actual fault, just information
    WARNING,     // Minor issue, normal operation can continue
    ERROR,       // Significant problem, but can continue with limitations
    CRITICAL,    // Major problem, requires recovery action
    FATAL        // System cannot continue, require restart
};

// Recovery action types
enum class RecoveryAction {
    NONE,                // No action needed
    RETRY_OPERATION,     // Try the operation again
    RESET_SUBSYSTEM,     // Reset the affected subsystem
    ENTER_SAFE_MODE,     // Enter a minimal safe operating mode
    REBOOT_SYSTEM        // Full system restart
};

// Fault record structure
struct FaultRecord {
    FaultType type;
    FaultSeverity severity;
    RecoveryAction action;
    unsigned long timestamp;
    String description;
    uint32_t errorCode;
    bool resolved;
    unsigned long resolutionTime;
};

class FaultHandler {
public:
    FaultHandler(StorageManager* storageManager, StateMachine* stateMachine = nullptr);

    // Initialize fault handler
    bool begin();

    // Report a fault
    void reportFault(FaultType type, FaultSeverity severity,
                     const String& description, uint32_t errorCode = 0);

    // Check if a fault was recently reported (for polling mechanisms)
    bool hasFault() const;

    // Get the count of active (unresolved) faults
    int getActiveFaultCount() const;

    // Get a list of all active faults
    std::vector<FaultRecord> getActiveFaults() const;

    // Mark a fault as resolved
    void resolveFault(FaultType type, uint32_t errorCode);

    // Reset all faults
    void resetAllFaults();

    // Get fault statistics
    int getTotalFaultCount() const;
    int getFaultCountByType(FaultType type) const;

    // Get a report of fault history
    String getFaultReport() const;

private:
    StorageManager* storageManager;
    StateMachine* stateMachine;

    std::vector<FaultRecord> faultHistory;
    bool hasActiveFault;

    // Determine recovery action based on fault type and severity
    RecoveryAction determineRecoveryAction(FaultType type, FaultSeverity severity);

    // Execute a recovery action
    void executeRecoveryAction(RecoveryAction action, FaultType type);

    // Log fault information
    void logFault(const FaultRecord& fault);

    // Check if we should trigger a state machine transition
    void checkStateTransition(const FaultRecord& fault);
};

#endif