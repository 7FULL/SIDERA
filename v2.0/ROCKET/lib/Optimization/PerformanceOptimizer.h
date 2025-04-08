/**
 * Performance Optimizer
 *
 * Adjusts system parameters for optimal performance in different flight phases
 */

#ifndef PERFORMANCE_OPTIMIZER_H
#define PERFORMANCE_OPTIMIZER_H

#include <Arduino.h>
#include "../HAL/StorageSystems/StorageManager.h"
#include "../StateMachine/StateMachine.h"
#include "ResourceMonitor.h"

// Optimization profiles for different flight phases
enum class OptimizationProfile {
    GROUND,       // Ground operations (idle, pre-launch)
    BOOST,        // Powered flight phase
    COAST,        // Coasting phase
    RECOVERY,     // Descent and recovery phase
    DEBUG,        // Maximum debugging and logging
    LOW_POWER     // Minimal power consumption
};

class PerformanceOptimizer {
public:
    PerformanceOptimizer(
            StorageManager* storageManager,
            StateMachine* stateMachine,
            ResourceMonitor* resourceMonitor
    );

    // Initialize optimizer
    bool begin();

    // Update optimization based on current state
    void update();

    // Manually set optimization profile
    void setProfile(OptimizationProfile profile);

    // Get current optimization profile
    OptimizationProfile getCurrentProfile() const;

    // Apply critical optimizations (emergency resource recovery)
    void applyCriticalOptimizations();

    // Enable/disable automatic optimization
    void setAutoOptimize(bool enable);

    // Set log verbosity
    void setLogVerbosity(uint8_t level);

    // Get optimization status
    String getOptimizationStatus() const;

private:
    StorageManager* storageManager;
    StateMachine* stateMachine;
    ResourceMonitor* resourceMonitor;

    OptimizationProfile currentProfile;
    bool autoOptimize;
    uint8_t verbosityLevel;

    // Apply specific profile optimizations
    void applyGroundProfile();
    void applyBoostProfile();
    void applyCoastProfile();
    void applyRecoveryProfile();
    void applyDebugProfile();
    void applyLowPowerProfile();

    // Map rocket state to optimization profile
    OptimizationProfile mapStateToProfile(RocketState state);

    // Log profile change
    void logProfileChange(OptimizationProfile oldProfile, OptimizationProfile newProfile);
};

#endif // PERFORMANCE_OPTIMIZER_H