/**
 * Specific Diagnostic Tests
 *
 * Implementation of specific tests for each subsystem
 */

#ifndef SPECIFIC_TESTS_H
#define SPECIFIC_TESTS_H

#include "DiagnosticTest.h"
#include "../HAL/BarometricSensors/BarometricSensorManager.h"
#include "../HAL/IMUSensors/IMUSensorManager.h"
#include "../HAL/GPSSensors/GPSSensorManager.h"
#include "../HAL/CommunicationSystems/LoRaSystem.h"
#include "../HAL/StorageSystems/StorageManager.h"
#include "../PowerManagement/PowerManager.h"
#include "../SensorFusion/SensorFusionSystem.h"
#include "TemperatureSensors/TemperatureSensorManager.h"

// Barometric sensor tests
class BarometricSensorTest : public DiagnosticTest {
public:
    BarometricSensorTest(BarometricSensorManager* manager);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    BarometricSensorManager* manager;
};

// IMU sensor tests
class IMUSensorTest : public DiagnosticTest {
public:
    IMUSensorTest(IMUSensorManager* manager);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    IMUSensorManager* manager;
};

// GPS sensor tests
class GPSSensorTest : public DiagnosticTest {
public:
    GPSSensorTest(GPSSensorManager* manager);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    GPSSensorManager* manager;
};

// LoRa communication tests
class LoRaCommunicationTest : public DiagnosticTest {
public:
    LoRaCommunicationTest(LoRaSystem* loraSystem);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    LoRaSystem* loraSystem;
};

// Storage tests
class StorageTest : public DiagnosticTest {
public:
    StorageTest(StorageManager* manager);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    StorageManager* manager;
};

// Battery tests
class BatteryTest : public DiagnosticTest {
public:
    BatteryTest(PowerManager* manager);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    PowerManager* manager;
};

// Sensor fusion tests
class SensorFusionTest : public DiagnosticTest {
public:
    SensorFusionTest(SensorFusionSystem* fusionSystem);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    SensorFusionSystem* fusionSystem;
};

// Temperature sensor tests
class TemperatureSensorTest : public DiagnosticTest {
public:
    TemperatureSensorTest(TemperatureSensorManager* manager);
    TestResult runTest() override;
    String getName() const override;
    String getDescription() const override;
    bool isCritical() const override;
    String getSubsystem() const override;

private:
    TemperatureSensorManager* manager;
};

#endif // SPECIFIC_TESTS_H