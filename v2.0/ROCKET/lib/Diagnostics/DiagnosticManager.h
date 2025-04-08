/**
 * Diagnostic Manager
 *
 * Manages diagnostic tests and reporting
 */

#ifndef DIAGNOSTIC_MANAGER_H
#define DIAGNOSTIC_MANAGER_H

#include "DiagnosticTest.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include <vector>

class DiagnosticManager {
public:
    DiagnosticManager(StorageManager* storageManager = nullptr);
    ~DiagnosticManager();

    // Add a test to the manager
    void addTest(DiagnosticTest* test);

    // Run all tests
    std::vector<TestResult> runAllTests();

    // Run tests for a specific subsystem
    std::vector<TestResult> runSubsystemTests(const String& subsystem);

    // Run critical tests only
    std::vector<TestResult> runCriticalTests();

    // Run a specific test by name
    TestResult runTest(const String& testName);

    // Check if all critical tests pass
    bool checkCriticalSystems();

    // Generate a diagnostic report
    String generateReport(const std::vector<TestResult>& results);

    // Get the last test results
    const std::vector<TestResult>& getLastResults() const;

    // Clear test results
    void clearResults();

    // Set verbose logging
    void setVerboseLogging(bool verbose);

    // Get all tests
    const std::vector<DiagnosticTest*>& getTests();

private:
    std::vector<DiagnosticTest*> tests;
    std::vector<TestResult> lastResults;
    StorageManager* storageManager;
    bool verboseLogging;

    // Log a test result
    void logTestResult(const TestResult& result);
};

#endif // DIAGNOSTIC_MANAGER_H