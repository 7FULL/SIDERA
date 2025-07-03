#ifndef DIAGNOSTIC_TEST_H
#define DIAGNOSTIC_TEST_H

#include <Arduino.h>
#include <vector>
#include <string>

// Test result structure
struct TestResult {
    bool passed;            // Overall pass/fail status
    String testName;        // Name of the test
    String description;     // Description of what was tested
    String errorMessage;    // Error message if failed
    int errorCode;          // Numeric error code
    float value;            // Measured value (if applicable)
    float expectedValue;    // Expected value (if applicable)
    float tolerance;        // Acceptable tolerance (if applicable)
    unsigned long timestamp;// When the test was performed
};

class DiagnosticTest {
public:
    virtual ~DiagnosticTest() = default;

    // Run the diagnostic test
    virtual TestResult runTest() = 0;

    // Get the name of the test
    virtual String getName() const = 0;

    // Get the description of the test
    virtual String getDescription() const = 0;

    // Check if the test is critical (must pass for flight)
    virtual bool isCritical() const = 0;

    // Get the subsystem this test belongs to
    virtual String getSubsystem() const = 0;

protected:
    // Helper method to create a test result
    TestResult createResult(bool passed, const String& errorMessage = "",
                            int errorCode = 0, float value = 0.0f,
                            float expectedValue = 0.0f, float tolerance = 0.0f) {
        TestResult result;
        result.passed = passed;
        result.testName = getName();
        result.description = getDescription();
        result.errorMessage = errorMessage;
        result.errorCode = errorCode;
        result.value = value;
        result.expectedValue = expectedValue;
        result.tolerance = tolerance;
        result.timestamp = millis();
        return result;
    }
};

#endif