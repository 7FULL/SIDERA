/**
 * Performance Optimizer Implementation
 */

#include "PerformanceOptimizer.h"

PerformanceOptimizer::PerformanceOptimizer(
        StorageManager* storageManager,
        StateMachine* stateMachine,
        ResourceMonitor* resourceMonitor
)
        : storageManager(storageManager),
          stateMachine(stateMachine),
          resourceMonitor(resourceMonitor),
          currentProfile(OptimizationProfile::GROUND),
          autoOptimize(true),
          verbosityLevel(1)
{
}

bool PerformanceOptimizer::begin() {
    // Initialize with default profile
    setProfile(OptimizationProfile::GROUND);

    // Log initialization
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Performance optimizer initialized");
    }

    return true;
}

void PerformanceOptimizer::update() {
    if (!autoOptimize || !stateMachine) {
        return;
    }

    // Get current rocket state
    RocketState currentState = stateMachine->getCurrentState();

    // Map to appropriate profile
    OptimizationProfile recommendedProfile = mapStateToProfile(currentState);

    // Check if profile needs to change
    if (recommendedProfile != currentProfile) {
        setProfile(recommendedProfile);
    }

    // Check resources and apply critical optimizations if needed
    if (resourceMonitor) {
        if (resourceMonitor->isMemoryLow() || resourceMonitor->isCpuHigh()) {
            applyCriticalOptimizations();
        }
    }
}

void PerformanceOptimizer::setProfile(OptimizationProfile profile) {
    if (profile == currentProfile) {
        return;  // No change needed
    }

    OptimizationProfile oldProfile = currentProfile;
    currentProfile = profile;

    // Apply profile-specific optimizations
    switch (profile) {
        case OptimizationProfile::GROUND:
            applyGroundProfile();
            break;
        case OptimizationProfile::BOOST:
            applyBoostProfile();
            break;
        case OptimizationProfile::COAST:
            applyCoastProfile();
            break;
        case OptimizationProfile::RECOVERY:
            applyRecoveryProfile();
            break;
        case OptimizationProfile::DEBUG:
            applyDebugProfile();
            break;
        case OptimizationProfile::LOW_POWER:
            applyLowPowerProfile();
            break;
    }

    // Log the change
    logProfileChange(oldProfile, currentProfile);
}

OptimizationProfile PerformanceOptimizer::getCurrentProfile() const {
    return currentProfile;
}

void PerformanceOptimizer::applyCriticalOptimizations() {
    if (storageManager) {
        storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM,
                                   "Applying critical optimizations");
    }

    // Reduce logging verbosity
    setLogVerbosity(0);  // Minimum logging

    // Disable non-critical tasks or features
    // This would depend on specific implementation details

    // Adjust sampling rates to minimum viable rates
    // This would call into other subsystems to reduce their activity
}

void PerformanceOptimizer::setAutoOptimize(bool enable) {
    autoOptimize = enable;

    if (storageManager) {
        char message[64];
        snprintf(message, sizeof(message), "Auto-optimization %s",
                 enable ? "enabled" : "disabled");
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }
}

void PerformanceOptimizer::setLogVerbosity(uint8_t level) {
    verbosityLevel = level;

    // Apply verbosity setting to all subsystems
    // This would depend on specific implementation details
}

String PerformanceOptimizer::getOptimizationStatus() const {
    String profileName;

    switch (currentProfile) {
        case OptimizationProfile::GROUND:
            profileName = "GROUND";
            break;
        case OptimizationProfile::BOOST:
            profileName = "BOOST";
            break;
        case OptimizationProfile::COAST:
            profileName = "COAST";
            break;
        case OptimizationProfile::RECOVERY:
            profileName = "RECOVERY";
            break;
        case OptimizationProfile::DEBUG:
            profileName = "DEBUG";
            break;
        case OptimizationProfile::LOW_POWER:
            profileName = "LOW_POWER";
            break;
        default:
            profileName = "UNKNOWN";
            break;
    }

    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "Profile: %s\n"
             "Auto-optimize: %s\n"
             "Verbosity: %u\n",
             profileName.c_str(),
             autoOptimize ? "ON" : "OFF",
             verbosityLevel);

    return String(buffer);
}

void PerformanceOptimizer::applyGroundProfile() {
    // Ground operations profile
    // - Moderate sensor sampling rate
    // - Higher telemetry rate
    // - More verbose logging
    setLogVerbosity(2);

    // This would call into other subsystems to adjust their parameters
}

void PerformanceOptimizer::applyBoostProfile() {
    // Boost profile
    // - Maximum sensor sampling rate
    // - High telemetry rate
    // - Reduced logging (focus on critical events)
    setLogVerbosity(1);

    // This would call into other subsystems to adjust their parameters
}

void PerformanceOptimizer::applyCoastProfile() {
    // Coast profile
    // - High sensor sampling rate
    // - Moderate telemetry rate
    // - Focus on altitude/apogee detection
    setLogVerbosity(1);

    // This would call into other subsystems to adjust their parameters
}

void PerformanceOptimizer::applyRecoveryProfile() {
    // Recovery profile
    // - Reduced sensor sampling rate
    // - Moderate telemetry rate
    // - Focus on GPS and recovery data
    setLogVerbosity(1);

    // This would call into other subsystems to adjust their parameters
}

void PerformanceOptimizer::applyDebugProfile() {
    // Debug profile
    // - Maximum logging and telemetry
    // - Full sensor data
    // - Comprehensive error handling
    setLogVerbosity(3);

    // This would call into other subsystems to adjust their parameters
}

void PerformanceOptimizer::applyLowPowerProfile() {
    // Low power profile
    // - Minimum sensor sampling
    // - Reduced telemetry
    // - Minimal logging
    setLogVerbosity(0);

    // This would call into other subsystems to adjust their parameters
}

OptimizationProfile PerformanceOptimizer::mapStateToProfile(RocketState state) {
    switch (state) {
        case RocketState::INIT:
        case RocketState::GROUND_IDLE:
            return OptimizationProfile::GROUND;

        case RocketState::READY:
            // If in countdown, use boost profile
            if (stateMachine->isInSubState(ReadySubState::COUNTDOWN)) {
                return OptimizationProfile::BOOST;
            } else {
                return OptimizationProfile::GROUND;
            }

        case RocketState::POWERED_FLIGHT:
            return OptimizationProfile::BOOST;

        case RocketState::COASTING:
        case RocketState::APOGEE:
            return OptimizationProfile::COAST;

        case RocketState::DESCENT:
        case RocketState::PARACHUTE_DESCENT:
        case RocketState::LANDED:
            return OptimizationProfile::RECOVERY;

        case RocketState::ERROR:
            // If in recovery mode, use low power
            if (stateMachine->isInSubState(ErrorSubState::RECOVERY_MODE)) {
                return OptimizationProfile::LOW_POWER;
            } else {
                return OptimizationProfile::DEBUG; // Debug mode for error diagnostics
            }

        default:
            return OptimizationProfile::GROUND;
    }
}

void PerformanceOptimizer::logProfileChange(OptimizationProfile oldProfile, OptimizationProfile newProfile) {
    if (!storageManager) {
        return;
    }

    const char* oldName;
    const char* newName;

    // Convert profiles to names
    switch (oldProfile) {
        case OptimizationProfile::GROUND: oldName = "GROUND"; break;
        case OptimizationProfile::BOOST: oldName = "BOOST"; break;
        case OptimizationProfile::COAST: oldName = "COAST"; break;
        case OptimizationProfile::RECOVERY: oldName = "RECOVERY"; break;
        case OptimizationProfile::DEBUG: oldName = "DEBUG"; break;
        case OptimizationProfile::LOW_POWER: oldName = "LOW_POWER"; break;
        default: oldName = "UNKNOWN"; break;
    }

    switch (newProfile) {
        case OptimizationProfile::GROUND: newName = "GROUND"; break;
        case OptimizationProfile::BOOST: newName = "BOOST"; break;
        case OptimizationProfile::COAST: newName = "COAST"; break;
        case OptimizationProfile::RECOVERY: newName = "RECOVERY"; break;
        case OptimizationProfile::DEBUG: newName = "DEBUG"; break;
        case OptimizationProfile::LOW_POWER: newName = "LOW_POWER"; break;
        default: newName = "UNKNOWN"; break;
    }

    char message[64];
    snprintf(message, sizeof(message), "Optimization profile changed: %s -> %s",
             oldName, newName);

    storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
}