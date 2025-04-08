/**
 * Fault Handler Implementation
 */

#include "FaultHandler.h"

FaultHandler::FaultHandler(StorageManager* storageManager, StateMachine* stateMachine)
        : storageManager(storageManager),
          stateMachine(stateMachine),
          hasActiveFault(false)
{
}

bool FaultHandler::begin() {
    // Initialize fault tracking
    faultHistory.clear();
    hasActiveFault = false;

    // Log initialization
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Fault handler initialized");
    }

    return true;
}

void FaultHandler::reportFault(FaultType type, FaultSeverity severity,
                               const String& description, uint32_t errorCode) {
    // Create new fault record
    FaultRecord fault;
    fault.type = type;
    fault.severity = severity;
    fault.timestamp = millis();
    fault.description = description;
    fault.errorCode = errorCode;
    fault.resolved = false;
    fault.resolutionTime = 0;

    // Determine appropriate recovery action
    fault.action = determineRecoveryAction(type, severity);

    // Add to history
    faultHistory.push_back(fault);

    // Set active fault flag
    hasActiveFault = true;

    // Log the fault
    logFault(fault);

    // Execute recovery action
    executeRecoveryAction(fault.action, fault.type);

    // Check if we should transition state machine
    checkStateTransition(fault);
}

bool FaultHandler::hasFault() const {
    return hasActiveFault;
}

int FaultHandler::getActiveFaultCount() const {
    int count = 0;
    for (const auto& fault : faultHistory) {
        if (!fault.resolved) {
            count++;
        }
    }
    return count;
}

std::vector<FaultRecord> FaultHandler::getActiveFaults() const {
    std::vector<FaultRecord> activeFaults;
    for (const auto& fault : faultHistory) {
        if (!fault.resolved) {
            activeFaults.push_back(fault);
        }
    }
    return activeFaults;
}

void FaultHandler::resolveFault(FaultType type, uint32_t errorCode) {
    bool foundFault = false;

    for (auto& fault : faultHistory) {
        if (fault.type == type && fault.errorCode == errorCode && !fault.resolved) {
            fault.resolved = true;
            fault.resolutionTime = millis();
            foundFault = true;

            // Log the resolution
            if (storageManager) {
                char message[64];
                snprintf(message, sizeof(message), "Fault resolved: Type=%d, Code=0x%08X",
                         static_cast<int>(type), errorCode);
                storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
            }
        }
    }

    // Update active fault flag if needed
    if (foundFault && getActiveFaultCount() == 0) {
        hasActiveFault = false;
    }
}

void FaultHandler::resetAllFaults() {
    // Mark all faults as resolved
    unsigned long currentTime = millis();
    for (auto& fault : faultHistory) {
        if (!fault.resolved) {
            fault.resolved = true;
            fault.resolutionTime = currentTime;
        }
    }

    hasActiveFault = false;

    // Log the reset
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "All faults reset");
    }
}

int FaultHandler::getTotalFaultCount() const {
    return faultHistory.size();
}

int FaultHandler::getFaultCountByType(FaultType type) const {
    int count = 0;
    for (const auto& fault : faultHistory) {
        if (fault.type == type) {
            count++;
        }
    }
    return count;
}

String FaultHandler::getFaultReport() const {
    String report = "FAULT REPORT\n";
    report += "============\n\n";

    report += "Total faults: " + String(faultHistory.size()) + "\n";
    report += "Active faults: " + String(getActiveFaultCount()) + "\n\n";

    // List active faults
    if (getActiveFaultCount() > 0) {
        report += "ACTIVE FAULTS:\n";
        for (const auto& fault : faultHistory) {
            if (!fault.resolved) {
                report += "- Type: " + String(static_cast<int>(fault.type)) +
                          ", Severity: " + String(static_cast<int>(fault.severity)) +
                          ", Code: 0x" + String(fault.errorCode, HEX) + "\n";
                report += "  " + fault.description + "\n";
                report += "  Reported: " + String(fault.timestamp) + " ms\n\n";
            }
        }
    }

    // List recent resolved faults (up to 5)
    int resolvedCount = 0;
    report += "RECENT RESOLVED FAULTS:\n";
    for (auto it = faultHistory.rbegin(); it != faultHistory.rend() && resolvedCount < 5; ++it) {
        if (it->resolved) {
            resolvedCount++;
            report += "- Type: " + String(static_cast<int>(it->type)) +
                      ", Severity: " + String(static_cast<int>(it->severity)) +
                      ", Code: 0x" + String(it->errorCode, HEX) + "\n";
            report += "  " + it->description + "\n";
            report += "  Resolved after: " +
                      String(it->resolutionTime - it->timestamp) + " ms\n\n";
        }
    }

    return report;
}

RecoveryAction FaultHandler::determineRecoveryAction(FaultType type, FaultSeverity severity) {
    // Determine recovery action based on fault type and severity

    // For fatal severity, always reboot
    if (severity == FaultSeverity::FATAL) {
        return RecoveryAction::REBOOT_SYSTEM;
    }

    // For critical severity, enter safe mode or reset subsystem
    if (severity == FaultSeverity::CRITICAL) {
        // For certain types, just reset the subsystem
        if (type == FaultType::SENSOR_TIMEOUT ||
            type == FaultType::COMMUNICATION_ERROR) {
            return RecoveryAction::RESET_SUBSYSTEM;
        }

        // For other critical issues, enter safe mode
        return RecoveryAction::ENTER_SAFE_MODE;
    }

    // For errors, try resetting the subsystem
    if (severity == FaultSeverity::ERROR) {
        if (type == FaultType::MEMORY_ERROR) {
            return RecoveryAction::ENTER_SAFE_MODE;  // Memory errors require safe mode
        }

        return RecoveryAction::RESET_SUBSYSTEM;
    }

    // For warnings, retry the operation
    if (severity == FaultSeverity::WARNING) {
        return RecoveryAction::RETRY_OPERATION;
    }

    // For info level, no action needed
    return RecoveryAction::NONE;
}

void FaultHandler::executeRecoveryAction(RecoveryAction action, FaultType type) {
    switch (action) {
        case RecoveryAction::NONE:
            // No action needed
            break;

        case RecoveryAction::RETRY_OPERATION:
            // Just log that we're retrying
            if (storageManager) {
                storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                           "Retrying operation after fault");
            }
            break;

        case RecoveryAction::RESET_SUBSYSTEM:
            // Reset the affected subsystem
            // This would require specific implementation for each subsystem
            if (storageManager) {
                char message[64];
                snprintf(message, sizeof(message), "Resetting subsystem for fault type %d",
                         static_cast<int>(type));
                storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM, message);
            }

            // Here we'd call specific reset functions based on fault type
            break;

        case RecoveryAction::ENTER_SAFE_MODE:
            // Enter safe mode operation
            if (storageManager) {
                storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM,
                                           "Entering safe mode due to fault");
            }

            // If state machine is available, force it to error state
            if (stateMachine) {
                stateMachine->processEvent(RocketEvent::ERROR_DETECTED);
            }

            break;

        case RecoveryAction::REBOOT_SYSTEM:
            // Log the reboot
            if (storageManager) {
                storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM,
                                           "SYSTEM REBOOT triggered by fault handler");

                // Flush storage to ensure the message is saved
                storageManager->flush();
            }

            // Give time for logs to be written
            delay(500);

            // Reboot the system
            //TODO
//            NVIC_SystemReset();
            break;
    }
}

void FaultHandler::logFault(const FaultRecord& fault) {
    if (!storageManager) {
        return;
    }

    // Determine log level based on severity
    LogLevel logLevel;
    switch (fault.severity) {
        case FaultSeverity::INFO:
            logLevel = LogLevel::INFO;
            break;
        case FaultSeverity::WARNING:
            logLevel = LogLevel::WARNING;
            break;
        case FaultSeverity::ERROR:
        case FaultSeverity::CRITICAL:
        case FaultSeverity::FATAL:
            logLevel = LogLevel::ERROR;
            break;
        default:
            logLevel = LogLevel::INFO;
            break;
    }

    // Create log message
    String message = "FAULT: " + fault.description;

    // Add details for more serious faults
    if (fault.severity >= FaultSeverity::ERROR) {
        char details[64];
        snprintf(details, sizeof(details), " (Type=%d, Code=0x%08X, Action=%d)",
                 static_cast<int>(fault.type),
                 fault.errorCode,
                 static_cast<int>(fault.action));

        message += details;
    }

    // Log the message
    storageManager->logMessage(logLevel, Subsystem::SYSTEM, message.c_str());
}

void FaultHandler::checkStateTransition(const FaultRecord& fault) {
    // If no state machine, we can't transition
    if (!stateMachine) {
        return;
    }

    // For critical and fatal faults, transition to error state
    if (fault.severity == FaultSeverity::CRITICAL ||
        fault.severity == FaultSeverity::FATAL) {

        stateMachine->processEvent(RocketEvent::ERROR_DETECTED);
    }
}