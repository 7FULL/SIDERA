/**
 * Diagnostic Manager Implementation
 */

#include "DiagnosticManager.h"

DiagnosticManager::DiagnosticManager(StorageManager* storageManager)
        : storageManager(storageManager), verboseLogging(false) {
}

DiagnosticManager::~DiagnosticManager() {
    // We don't delete tests here because they might be used elsewhere
}

void DiagnosticManager::addTest(DiagnosticTest* test) {
    if (test) {
        tests.push_back(test);
    }
}



std::vector<TestResult> DiagnosticManager::runAllTests() {
    lastResults.clear();

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Starting diagnostic tests");
    }

    for (auto test : tests) {
        TestResult result = test->runTest();
        lastResults.push_back(result);
        logTestResult(result);
    }

    if (storageManager) {
        char message[64];
        int passCount = 0;
        for (const auto& result : lastResults) {
            if (result.passed) passCount++;
        }

        snprintf(message, sizeof(message), "Diagnostics complete: %d/%d tests passed",
                 passCount, (int)lastResults.size());
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }

    return lastResults;
}

std::vector<TestResult> DiagnosticManager::runSubsystemTests(const String& subsystem) {
    std::vector<TestResult> results;

    if (storageManager) {
        char message[64];
        snprintf(message, sizeof(message), "Starting %s subsystem tests",
                 subsystem.c_str());
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }

    for (auto test : tests) {
        if (test->getSubsystem() == subsystem) {
            TestResult result = test->runTest();
            results.push_back(result);
            logTestResult(result);
        }
    }

    if (storageManager) {
        char message[64];
        int passCount = 0;
        for (const auto& result : results) {
            if (result.passed) passCount++;
        }

        snprintf(message, sizeof(message), "%s diagnostics: %d/%d tests passed",
                 subsystem.c_str(), passCount, (int)results.size());
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }

    lastResults = results;
    return results;
}

std::vector<TestResult> DiagnosticManager::runCriticalTests() {
    std::vector<TestResult> results;

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "Starting critical system tests");
    }

    for (auto test : tests) {
        if (test->isCritical()) {
            TestResult result = test->runTest();
            results.push_back(result);
            logTestResult(result);
        }
    }

    if (storageManager) {
        char message[64];
        int passCount = 0;
        for (const auto& result : results) {
            if (result.passed) passCount++;
        }

        snprintf(message, sizeof(message), "Critical diagnostics: %d/%d tests passed",
                 passCount, (int)results.size());
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM, message);
    }

    lastResults = results;
    return results;
}

TestResult DiagnosticManager::runTest(const String& testName) {
    for (auto test : tests) {
        if (test->getName() == testName) {
            TestResult result = test->runTest();
            logTestResult(result);

            // Store the result
            bool found = false;
            for (size_t i = 0; i < lastResults.size(); i++) {
                if (lastResults[i].testName == testName) {
                    lastResults[i] = result;
                    found = true;
                    break;
                }
            }

            if (!found) {
                lastResults.push_back(result);
            }

            return result;
        }
    }

    // Test not found
    TestResult notFoundResult;
    notFoundResult.passed = false;
    notFoundResult.testName = testName;
    notFoundResult.description = "Test not found";
    notFoundResult.errorMessage = "Test with specified name does not exist";
    notFoundResult.errorCode = -1;
    notFoundResult.timestamp = millis();

    return notFoundResult;
}

bool DiagnosticManager::checkCriticalSystems() {
    std::vector<TestResult> criticalResults = runCriticalTests();

    // Check if all critical tests passed
    for (const auto& result : criticalResults) {
        if (!result.passed) {
            if (storageManager) {
                char message[64];
                snprintf(message, sizeof(message), "CRITICAL TEST FAILED: %s",
                         result.testName.c_str());
                storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM, message);
            }
            return false;
        }
    }

    if (storageManager) {
        storageManager->logMessage(LogLevel::INFO, Subsystem::SYSTEM,
                                   "All critical systems passed diagnostics");
    }

    return true;
}

String DiagnosticManager::generateReport(const std::vector<TestResult>& results) {
    String report = "DIAGNOSTIC REPORT\n";
    report += "=================\n\n";

    int passCount = 0;
    int failCount = 0;

    for (const auto& result : results) {
        if (result.passed) {
            passCount++;
        } else {
            failCount++;
        }

        report += "Test: " + result.testName + "\n";
        report += "Subsystem: " + result.description + "\n";
        report += "Status: " + String(result.passed ? "PASS" : "FAIL") + "\n";

        if (!result.passed) {
            report += "Error: " + result.errorMessage + "\n";
            report += "Error Code: " + String(result.errorCode) + "\n";
        }

        if (result.value != 0.0f || result.expectedValue != 0.0f) {
            report += "Measured Value: " + String(result.value) + "\n";
            if (result.expectedValue != 0.0f) {
                report += "Expected Value: " + String(result.expectedValue);
                if (result.tolerance != 0.0f) {
                    report += " ± " + String(result.tolerance);
                }
                report += "\n";
            }
        }

        report += "Timestamp: " + String(result.timestamp) + "\n\n";
    }

    report += "Summary: " + String(passCount) + " passed, " +
              String(failCount) + " failed\n";

    return report;
}

const std::vector<TestResult>& DiagnosticManager::getLastResults() const {
    return lastResults;
}

const std::vector<DiagnosticTest*>& DiagnosticManager::getTests() {
    return tests;
}

void DiagnosticManager::clearResults() {
    lastResults.clear();
}

void DiagnosticManager::setVerboseLogging(bool verbose) {
    verboseLogging = verbose;
}

void DiagnosticManager::logTestResult(const TestResult& result) {
    if (!storageManager) {
        return;
    }

    // Always log failures
    if (!result.passed) {
        char message[64];
        snprintf(message, sizeof(message), "TEST FAILED: %s - %s",
                 result.testName.c_str(), result.errorMessage.c_str());
        storageManager->logMessage(LogLevel::ERROR, Subsystem::SYSTEM, message);
    }
        // Log passes only if verbose
    else if (verboseLogging) {
        char message[64];
        snprintf(message, sizeof(message), "Test passed: %s",
                 result.testName.c_str());
        storageManager->logMessage(LogLevel::DEBUG, Subsystem::SYSTEM, message);
    }
}