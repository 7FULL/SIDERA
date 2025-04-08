/**
 * Resource Monitor
 *
 * Tracks system resources and performance metrics
 */

#ifndef RESOURCE_MONITOR_H
#define RESOURCE_MONITOR_H

#include <Arduino.h>
#include "../HAL/StorageSystems/StorageManager.h"
#include <FreeRTOS.h>
#include <task.h>

class ResourceMonitor {
public:
    ResourceMonitor(StorageManager* storageManager = nullptr);

    // Initialize monitor
    bool begin();

    // Update resource metrics
    void update();

    // Get current free heap memory
    size_t getFreeHeap() const;

    // Get minimum free heap seen
    size_t getMinFreeHeap() const;

    // Get current free stack for a task
    size_t getTaskFreeStack(TaskHandle_t task) const;

    // Get CPU usage as percentage (if supported)
    float getCpuUsage() const;

    // Get loop execution time in microseconds
    unsigned long getLoopTime() const;

    // Set warning thresholds
    void setMemoryWarningThreshold(size_t bytes);
    void setCpuWarningThreshold(float percentage);

    // Check if resources are at warning levels
    bool isMemoryLow() const;
    bool isCpuHigh() const;

    // Log current resource usage
    void logResourceUsage();

    // Get a textual summary of resource usage
    String getResourceSummary() const;

private:
    StorageManager* storageManager;

    // Resource metrics
    size_t freeHeap;
    size_t minFreeHeap;
    float cpuUsage;
    unsigned long loopTime;
    unsigned long lastLoopStartTime;

    // Warning thresholds
    size_t memoryWarningThreshold;
    float cpuWarningThreshold;

    // Last log time
    unsigned long lastLogTime;

    // CPU usage calculation
    void calculateCpuUsage();
};

#endif // RESOURCE_MONITOR_H