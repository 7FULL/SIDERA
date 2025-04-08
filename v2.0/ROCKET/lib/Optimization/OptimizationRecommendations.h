/**
 * Optimization Recommendations
 *
 * Specific optimizations for different parts of the rocket system
 */

#ifndef OPTIMIZATION_RECOMMENDATIONS_H
#define OPTIMIZATION_RECOMMENDATIONS_H

#include <Arduino.h>
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include "../SensorFusion/SensorFusionSystem.h"

class OptimizationRecommendations {
public:
    // Memory optimization recommendations
    static void applyMemoryOptimizations();

    // CPU usage optimization recommendations
    static void applyCpuOptimizations();

    // Sensor optimizations
    static void optimizeBarometricSensors(BarometricSensorManager* manager);
    static void optimizeIMUSensors(IMUSensorManager* manager);
    static void optimizeGPSSensors(GPSSensorManager* manager);

    // Communication optimizations
    static void optimizeCommunication(LoRaSystem* loraSystem);

    // Storage optimizations
    static void optimizeStorage(StorageManager* storageManager);

    // Sensor fusion optimizations
    static void optimizeSensorFusion(SensorFusionSystem* fusionSystem);

    // Code optimization patterns
    static void demonstrateOptimizedPatterns();
};

#endif // OPTIMIZATION_RECOMMENDATIONS_H