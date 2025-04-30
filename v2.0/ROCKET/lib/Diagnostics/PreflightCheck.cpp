/**
 * Pre-flight Check System Implementation
 */

#include "PreflightCheck.h"

PreflightCheckSystem::PreflightCheckSystem(
        DiagnosticManager* diagnosticManager,
        StorageManager* storageManager
)
        : diagnosticManager(diagnosticManager),
          storageManager(storageManager),
          currentStatus(PreflightStatus::NOT_STARTED),
          currentPhase(PreflightPhase::POWER) {
}

bool PreflightCheckSystem::begin() {
    if (!diagnosticManager) {
        return false;
    }

    currentStatus = PreflightStatus::NOT_STARTED;
    currentPhase = PreflightPhase::POWER;
    checkResults.clear();

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Pre-flight check system initialized");
    }

    return true;
}

PreflightStatus PreflightCheckSystem::runPreflightChecks() {
    Serial.println("Running pre-flight checks...");

    if (!diagnosticManager) {
        currentStatus = PreflightStatus::FAILED;
        return currentStatus;
    }

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Starting pre-flight check sequence");
    }

    currentStatus = PreflightStatus::IN_PROGRESS;
    checkResults.clear();

    // Run each phase in sequence
    PreflightStatus phaseStatus;

    // Phase 1: Power systems
    currentPhase = PreflightPhase::POWER;
    phaseStatus = checkPowerSystems();
    if (phaseStatus == PreflightStatus::FAILED) {
        currentStatus = PreflightStatus::FAILED;
        logStatus(currentStatus, currentPhase);
        return currentStatus;
    }

    //TODO
    // Phase 2: Sensor systems
//    currentPhase = PreflightPhase::SENSORS;
//    phaseStatus = checkSensorSystems();
//    if (phaseStatus == PreflightStatus::FAILED) {
//        currentStatus = PreflightStatus::FAILED;
//        logStatus(currentStatus, currentPhase);
//        return currentStatus;
//    }

    // Phase 3: Communication systems
    currentPhase = PreflightPhase::COMMUNICATION;
    phaseStatus = checkCommunicationSystems();
    if (phaseStatus == PreflightStatus::FAILED) {
        currentStatus = PreflightStatus::FAILED;
        logStatus(currentStatus, currentPhase);
        return currentStatus;
    }

    // Phase 4: Storage systems
//    currentPhase = PreflightPhase::STORAGE;
//    phaseStatus = checkStorageSystems();
//    if (phaseStatus == PreflightStatus::FAILED) {
//        currentStatus = PreflightStatus::FAILED;
//        logStatus(currentStatus, currentPhase);
//        return currentStatus;
//    }

    // Phase 5: Fusion systems
    currentPhase = PreflightPhase::FUSION;
    phaseStatus = checkFusionSystems();
    if (phaseStatus == PreflightStatus::FAILED) {
        currentStatus = PreflightStatus::FAILED;
        logStatus(currentStatus, currentPhase);
        return currentStatus;
    }

    // Phase 6: Final checks
    currentPhase = PreflightPhase::FINAL;
    phaseStatus = performFinalChecks();

    // Set final status
    currentStatus = phaseStatus;
    logStatus(currentStatus, currentPhase);

    return currentStatus;
}

PreflightStatus PreflightCheckSystem::runCheckPhase(PreflightPhase phase) {
    if (!diagnosticManager) {
        return PreflightStatus::FAILED;
    }

    currentPhase = phase;

    PreflightStatus phaseStatus;

    switch (phase) {
        case PreflightPhase::POWER:
            phaseStatus = checkPowerSystems();
            break;

        case PreflightPhase::SENSORS:
            phaseStatus = checkSensorSystems();
            break;

        case PreflightPhase::COMMUNICATION:
            phaseStatus = checkCommunicationSystems();
            break;

        case PreflightPhase::STORAGE:
            phaseStatus = checkStorageSystems();
            break;

        case PreflightPhase::FUSION:
            phaseStatus = checkFusionSystems();
            break;

        case PreflightPhase::FINAL:
            phaseStatus = performFinalChecks();
            break;

        default:
            phaseStatus = PreflightStatus::FAILED;
            break;
    }

    logStatus(phaseStatus, phase);
    return phaseStatus;
}

PreflightStatus PreflightCheckSystem::getStatus() const {
    return currentStatus;
}

PreflightPhase PreflightCheckSystem::getCurrentPhase() const {
    return currentPhase;
}

const std::vector<TestResult>& PreflightCheckSystem::getCheckResults() const {
    return checkResults;
}

String PreflightCheckSystem::generatePreflightReport() {
    String report = "PRE-FLIGHT CHECK REPORT\n";
    report += "========================\n\n";

    report += "Overall Status: ";
    switch (currentStatus) {
        case PreflightStatus::NOT_STARTED:
            report += "NOT STARTED\n";
            break;
        case PreflightStatus::IN_PROGRESS:
            report += "IN PROGRESS\n";
            break;
        case PreflightStatus::PASSED:
            report += "PASSED\n";
            break;
        case PreflightStatus::FAILED:
            report += "FAILED\n";
            break;
        case PreflightStatus::WARNING:
            report += "WARNING\n";
            break;
    }

    report += "Current Phase: ";
    switch (currentPhase) {
        case PreflightPhase::POWER:
            report += "POWER SYSTEMS\n";
            break;
        case PreflightPhase::SENSORS:
            report += "SENSOR SYSTEMS\n";
            break;
        case PreflightPhase::COMMUNICATION:
            report += "COMMUNICATION SYSTEMS\n";
            break;
        case PreflightPhase::STORAGE:
            report += "STORAGE SYSTEMS\n";
            break;
        case PreflightPhase::FUSION:
            report += "FUSION SYSTEMS\n";
            break;
        case PreflightPhase::FINAL:
            report += "FINAL CHECKS\n";
            break;
    }

    report += "\nDetailed Results:\n";
    report += "----------------\n\n";

    // Group results by subsystem
    std::vector<String> subsystems;
    for (const auto& result : checkResults) {
        bool found = false;
        for (const auto& subsystem : subsystems) {
            if (subsystem == result.testName) {
                found = true;
                break;
            }
        }

        if (!found) {
            subsystems.push_back(result.testName);
        }
    }

    for (const auto& subsystem : subsystems) {
        report += "Subsystem: " + subsystem + "\n";

        int passCount = 0;
        int failCount = 0;

        for (const auto& result : checkResults) {
            if (result.testName == subsystem) {
                if (result.passed) {
                    passCount++;
                } else {
                    failCount++;
                    report += "  FAIL: " + result.errorMessage + "\n";
                }
            }
        }

        report += "  Summary: " + String(passCount) + " passed, " +
                  String(failCount) + " failed\n\n";
    }

    report += "Ready to fly: " + String(isReadyToFly() ? "YES" : "NO") + "\n";

    return report;
}

bool PreflightCheckSystem::isReadyToFly() const {
    return currentStatus == PreflightStatus::PASSED;
}

void PreflightCheckSystem::reset() {
    currentStatus = PreflightStatus::NOT_STARTED;
    currentPhase = PreflightPhase::POWER;
    checkResults.clear();

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Pre-flight checks reset");
    }
}

PreflightStatus PreflightCheckSystem::checkPowerSystems() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Checking power systems...");
    }

    // Run power-related tests
    std::vector<TestResult> powerResults =
            diagnosticManager->runSubsystemTests("POWER");

    // Add results to our collection
    checkResults.insert(checkResults.end(), powerResults.begin(), powerResults.end());

    // Check if any critical tests failed
    bool criticalFailure = false;
    for (const auto& result : powerResults) {
        if (!result.passed) {
            criticalFailure = true;
            break;
        }
    }

    if (criticalFailure) {
        return PreflightStatus::FAILED;
    }

    return PreflightStatus::PASSED;
}

PreflightStatus PreflightCheckSystem::checkSensorSystems() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Checking sensor systems...");
    }

    // Run barometer tests
    std::vector<TestResult> baroResults =
            diagnosticManager->runSubsystemTests("BAROMETER");
    checkResults.insert(checkResults.end(), baroResults.begin(), baroResults.end());

    // Run IMU tests
    std::vector<TestResult> imuResults =
            diagnosticManager->runSubsystemTests("IMU");
    checkResults.insert(checkResults.end(), imuResults.begin(), imuResults.end());

    // Run GPS tests (non-critical)
    std::vector<TestResult> gpsResults =
            diagnosticManager->runSubsystemTests("GPS");
    checkResults.insert(checkResults.end(), gpsResults.begin(), gpsResults.end());

    // Check if any critical barometer or IMU tests failed
    bool criticalFailure = false;

    for (const auto& result : baroResults) {
        if (!result.passed) {
            criticalFailure = true;
            break;
        }
    }

    for (const auto& result : imuResults) {
        if (!result.passed) {
            criticalFailure = true;
            break;
        }
    }

    if (criticalFailure) {
        return PreflightStatus::FAILED;
    }

    // GPS failures are warnings, not critical failures
    bool hasWarnings = false;
    for (const auto& result : gpsResults) {
        if (!result.passed) {
            hasWarnings = true;
            break;
        }
    }

    if (hasWarnings) {
        return PreflightStatus::WARNING;
    }

    return PreflightStatus::PASSED;
}

PreflightStatus PreflightCheckSystem::checkCommunicationSystems() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Checking communication systems...");
    }

    // Run communication tests
    std::vector<TestResult> commResults =
            diagnosticManager->runSubsystemTests("COMMUNICATION");

    // Add results to our collection
    checkResults.insert(checkResults.end(), commResults.begin(), commResults.end());

    // Check if any critical tests failed
    bool criticalFailure = false;
    for (const auto& result : commResults) {
        if (!result.passed) {
            criticalFailure = true;
            break;
        }
    }

    if (criticalFailure) {
        return PreflightStatus::FAILED;
    }

    return PreflightStatus::PASSED;
}

PreflightStatus PreflightCheckSystem::checkStorageSystems() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Checking storage systems...");
    }

    // Run storage tests
    std::vector<TestResult> storageResults =
            diagnosticManager->runSubsystemTests("STORAGE");

    // Add results to our collection
    checkResults.insert(checkResults.end(), storageResults.begin(), storageResults.end());

    // Check if any critical tests failed
    bool criticalFailure = false;
    for (const auto& result : storageResults) {
        if (!result.passed) {
            criticalFailure = true;
            break;
        }
    }

    if (criticalFailure) {
        return PreflightStatus::FAILED;
    }

    return PreflightStatus::PASSED;
}

PreflightStatus PreflightCheckSystem::checkFusionSystems() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Checking sensor fusion systems...");
    }

    // Run fusion tests
    std::vector<TestResult> fusionResults =
            diagnosticManager->runSubsystemTests("FUSION");

    // Add results to our collection
    checkResults.insert(checkResults.end(), fusionResults.begin(), fusionResults.end());

    // Check if any critical tests failed
    bool criticalFailure = false;
    for (const auto& result : fusionResults) {
        if (!result.passed) {
            criticalFailure = true;
            break;
        }
    }

    if (criticalFailure) {
        return PreflightStatus::FAILED;
    }

    return PreflightStatus::PASSED;
}

PreflightStatus PreflightCheckSystem::performFinalChecks() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Performing final checks...");
    }

    // Count failures and warnings
    int failureCount = 0;
    int warningCount = 0;

    for (const auto& result : checkResults) {
        if (!result.passed) {
            // Check if this is a critical test
            for (auto test : diagnosticManager->getTests()) {
                if (test->getName() == result.testName && test->isCritical()) {
                    failureCount++;
                    break;
                } else if (test->getName() == result.testName) {
                    warningCount++;
                    break;
                }
            }
        }
    }

    // Determine final status
    if (failureCount > 0) {
        if (storageManager) {
            char message[64];
            snprintf(message, sizeof(message), "Pre-flight checks failed: %d critical failures",
                     failureCount);
            storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM, message);
        }
        return PreflightStatus::FAILED;
    } else if (warningCount > 0) {
        if (storageManager) {
            char message[64];
            snprintf(message, sizeof(message), "Pre-flight checks have warnings: %d non-critical issues",
                     warningCount);
            storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM, message);
        }
        return PreflightStatus::WARNING;
    } else {
        if (storageManager) {
            storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                       "All pre-flight checks passed!");
        }
        return PreflightStatus::PASSED;
    }
}

void PreflightCheckSystem::logStatus(PreflightStatus status, PreflightPhase phase) {
    if (!storageManager) {
        return;
    }

    char message[64];
    const char* phaseStr;

    switch (phase) {
        case PreflightPhase::POWER:
            phaseStr = "POWER";
            break;
        case PreflightPhase::SENSORS:
            phaseStr = "SENSORS";
            break;
        case PreflightPhase::COMMUNICATION:
            phaseStr = "COMMUNICATION";
            break;
        case PreflightPhase::STORAGE:
            phaseStr = "STORAGE";
            break;
        case PreflightPhase::FUSION:
            phaseStr = "FUSION";
            break;
        case PreflightPhase::FINAL:
            phaseStr = "FINAL";
            break;
        default:
            phaseStr = "UNKNOWN";
            break;
    }

    switch (status) {
        case PreflightStatus::PASSED:
            snprintf(message, sizeof(message), "Pre-flight check phase %s: PASSED", phaseStr);
            storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
            break;

        case PreflightStatus::FAILED:
            snprintf(message, sizeof(message), "Pre-flight check phase %s: FAILED", phaseStr);
            storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM, message);
            break;

        case PreflightStatus::WARNING:
            snprintf(message, sizeof(message), "Pre-flight check phase %s: WARNING", phaseStr);
            storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM, message);
            break;

        default:
            break;
    }
}