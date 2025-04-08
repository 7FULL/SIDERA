/**
 * Resource Monitor Implementation
 */

#include "ResourceMonitor.h"

ResourceMonitor::ResourceMonitor(StorageManager* storageManager)
        : storageManager(storageManager),
          freeHeap(0),
          minFreeHeap(SIZE_MAX),
          cpuUsage(0.0f),
          loopTime(0),
          lastLoopStartTime(0),
          memoryWarningThreshold(1024),   // Default to 1KB warning
          cpuWarningThreshold(90.0f),     // Default to 90% CPU warning
          lastLogTime(0)
{
}

bool ResourceMonitor::begin() {
    // Initialize metrics
    freeHeap = rp2040.getFreeHeap();
    minFreeHeap = freeHeap;
    cpuUsage = 0.0f;
    loopTime = 0;
    lastLoopStartTime = micros();
    lastLogTime = millis();

    // Log initialization
    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Resource monitor initialized");
    }

    return true;
}

void ResourceMonitor::update() {
    // Calculate loop time
    unsigned long currentTime = micros();
    loopTime = currentTime - lastLoopStartTime;
    lastLoopStartTime = currentTime;

    // Update memory metrics
    freeHeap = rp2040.getFreeHeap();
    if (freeHeap < minFreeHeap) {
        minFreeHeap = freeHeap;
    }

    // Calculate CPU usage
    calculateCpuUsage();

    // Log resource usage periodically
    if (millis() - lastLogTime >= 60000) {  // Every minute
        lastLogTime = millis();
        logResourceUsage();
    }

    // Check for warnings
    if (isMemoryLow() && storageManager) {
        char message[64];
        snprintf(message, sizeof(message), "LOW MEMORY WARNING: %u bytes free",
                 (unsigned int)freeHeap);
        storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM, message);
    }

    if (isCpuHigh() && storageManager) {
        char message[64];
        snprintf(message, sizeof(message), "HIGH CPU WARNING: %.1f%% usage", cpuUsage);
        storageManager->logMessage(LogLevel::WARNING, Subsystem::SYSTEM, message);
    }
}

size_t ResourceMonitor::getFreeHeap() const {
    return freeHeap;
}

size_t ResourceMonitor::getMinFreeHeap() const {
    return minFreeHeap;
}

size_t ResourceMonitor::getTaskFreeStack(TaskHandle_t task) const {
    if (task == nullptr) {
        return 0;
    }

    return uxTaskGetStackHighWaterMark(task);
}

float ResourceMonitor::getCpuUsage() const {
    return cpuUsage;
}

unsigned long ResourceMonitor::getLoopTime() const {
    return loopTime;
}

void ResourceMonitor::setMemoryWarningThreshold(size_t bytes) {
    memoryWarningThreshold = bytes;
}

void ResourceMonitor::setCpuWarningThreshold(float percentage) {
    cpuWarningThreshold = percentage;
}

bool ResourceMonitor::isMemoryLow() const {
    return freeHeap < memoryWarningThreshold;
}

bool ResourceMonitor::isCpuHigh() const {
    return cpuUsage > cpuWarningThreshold;
}

void ResourceMonitor::logResourceUsage() {
    if (!storageManager) {
        return;
    }

    char message[64];
    snprintf(message, sizeof(message), "Memory: %u free, %u min, CPU: %.1f%%, Loop: %lu us",
             (unsigned int)freeHeap, (unsigned int)minFreeHeap,
             cpuUsage, loopTime);

    storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
}

String ResourceMonitor::getResourceSummary() const {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "Free Heap: %u bytes\n"
             "Min Free Heap: %u bytes\n"
             "CPU Usage: %.1f%%\n"
             "Loop Time: %lu us\n",
             (unsigned int)freeHeap, (unsigned int)minFreeHeap,
             cpuUsage, loopTime);

    return String(buffer);
}

void ResourceMonitor::calculateCpuUsage() {
    // This is just an approximation, as accurate CPU measurement
    // depends on specific hardware capabilities

    // Calculate idle time based on how long we spend processing in update()
    // Real implementation would need more sophisticated approach
    static unsigned long totalTime = 0;
    static unsigned long busyTime = 0;

    // Total time is the loop interval
    totalTime += loopTime;

    // Busy time is how long spent in processing (measured from this function call)
    unsigned long startTime = micros();

    // Here we'd perform a sample processing task to measure system responsiveness
    // But for now just estimate from our loop time
    busyTime += loopTime / 2;  // Assume half of loop time is "busy"

    // Reset counters periodically to avoid overflow and get fresh measurements
    if (totalTime > 1000000) {  // Reset every second of measurement
        cpuUsage = (busyTime * 100.0f) / totalTime;
        totalTime = 0;
        busyTime = 0;
    }
}