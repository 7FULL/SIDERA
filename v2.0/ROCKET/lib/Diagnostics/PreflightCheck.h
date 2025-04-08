/**
 * Pre-flight Check System
 *
 * Comprehensive verification system for pre-launch checks
 */

#ifndef PREFLIGHT_CHECK_H
#define PREFLIGHT_CHECK_H

#include "DiagnosticManager.h"
#include "SpecificTests.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include <vector>

// Pre-flight check status
enum class PreflightStatus {
    NOT_STARTED,
    IN_PROGRESS,
    PASSED,
    FAILED,
    WARNING
};

// Pre-flight check phase
enum class PreflightPhase {
    POWER,
    SENSORS,
    COMMUNICATION,
    STORAGE,
    FUSION,
    FINAL
};

class PreflightCheckSystem {
public:
    PreflightCheckSystem(
            DiagnosticManager* diagnosticManager,
            StorageManager* storageManager = nullptr
    );

    // Initialize the pre-flight check system
    bool begin();

    // Run complete pre-flight check sequence
    PreflightStatus runPreflightChecks();

    // Run a specific phase of pre-flight checks
    PreflightStatus runCheckPhase(PreflightPhase phase);

    // Get current pre-flight status
    PreflightStatus getStatus() const;

    // Get current pre-flight phase
    PreflightPhase getCurrentPhase() const;

    // Get detailed check results
    const std::vector<TestResult>& getCheckResults() const;

    // Generate pre-flight report
    String generatePreflightReport();

    // Check if rocket is ready to fly
    bool isReadyToFly() const;

    // Reset pre-flight checks
    void reset();

private:
    DiagnosticManager* diagnosticManager;
    StorageManager* storageManager;

    PreflightStatus currentStatus;
    PreflightPhase currentPhase;
    std::vector<TestResult> checkResults;

    // Phase handlers
    PreflightStatus checkPowerSystems();
    PreflightStatus checkSensorSystems();
    PreflightStatus checkCommunicationSystems();
    PreflightStatus checkStorageSystems();
    PreflightStatus checkFusionSystems();
    PreflightStatus performFinalChecks();

    // Log pre-flight status
    void logStatus(PreflightStatus status, PreflightPhase phase);
};

#endif // PREFLIGHT_CHECK_H