/**
 * Rocket Control System - RP2040 Version
 * Main entry point
 *
 * This file initializes the hardware, sets up FreeRTOS tasks
 * and starts the scheduler on both cores.
 */

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "Config.h"
#include "TaskConfig.h"
#include "PinDefinitions.h"

#include "../lib/SensorFusion/SensorFusionSystem.h"
#include "../lib/Diagnostics/DiagnosticManager.h"
#include "../lib/Diagnostics/SpecificTests.h"
#include "../lib/Diagnostics/PreflightCheck.h"

#include "../lib/Optimization/ResourceMonitor.h"
#include "../lib/Optimization/PerformanceOptimizer.h"
#include "../lib/Optimization/FaultHandler.h"

// Include HAL components
#include "../lib/HAL/BarometricSensors/BarometricSensorManager.h"
#include "../lib/HAL/BarometricSensors/BMP388Sensor.h"
#include "../lib/HAL/BarometricSensors/MPL3115A2Sensor.h"

#include "../lib/HAL/IMUSensors/IMUSensorManager.h"
#include "../lib/HAL/IMUSensors/BMI088Sensor.h"
#include "../lib/HAL/IMUSensors/ADXL375Sensor.h"

#include "../lib/HAL/TemperatureSensors/DS18B20Sensor.h"

#include "../lib/HAL/GPSSensors/GPSSensorManager.h"
#include "../lib/HAL/GPSSensors/L76KBGPSSensor.h"
#include "../lib/HAL/GPSSensors/ATGM336HGPSSensor.h"

#include "../lib/PowerManagement/PowerManager.h"
#include "../lib/PowerManagement/PowerController.h"

#include "../lib/HAL/CommunicationSystems/LoRaSystem.h"
#include "../lib/HAL/CommunicationSystems/TelemetrySerializer.h"
#include "../lib/HAL/CommunicationSystems/CommandSerializer.h"

#include "../lib/HAL/StorageSystems/StorageManager.h"
#include "../lib/HAL/StorageSystems/FlashStorage.h"
#include "../lib/HAL/StorageSystems/SDStorage.h"

#include "../lib/Communication/RocketProtocol.h"
#include "../lib/Communication/CommandHandler.h"

// Include State Machine
#include "../lib/StateMachine/StateMachine.h"
#include "../lib/StateMachine/StateHandlers.h"

// Forward declarations of core tasks
void Core0Task(void *pvParameters);
void Core1Task(void *pvParameters);

// Synchronization primitives
SemaphoreHandle_t xSensorDataMutex;
SemaphoreHandle_t xSDCardMutex;
SemaphoreHandle_t xTelemetrySemaphore;

// Initialization status flags
volatile bool sensorsInitialized = false;
volatile bool storageInitialized = false;
volatile bool communicationInitialized = false;
volatile bool apogeeDetectedFlag = false;

// Hardware interfaces
SPIClass& vspi = SPI;  // Use default SPI for SD card
SPIClass& hspi = SPI1; // Use SPI1 for LoRa and Flash

// Sensor managers
BarometricSensorManager baroManager;
IMUSensorManager imuManager;
GPSSensorManager gpsManager;

// Optimization components
ResourceMonitor* resourceMonitor;
PerformanceOptimizer* performanceOptimizer;
FaultHandler* faultHandler;

// Storage systems
FlashStorage* flashStorage;
SDStorage* sdStorage;
StorageManager storageManager;

// Power management
PowerManager* powerManager;
PowerController* powerController;

// Diagnostic manager
DiagnosticManager* diagnosticManager;
PreflightCheckSystem* preflightSystem;

// Command handler
CommandHandler* commandHandler;

// Communication system
LoRaSystem* loraSystem;

// Fusion system
SensorFusionSystem* fusionSystem;

// State machine
StateMachine stateMachine;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(2000); // Give time for serial monitor to connect
    Serial.println("Rocket Control System - RP2040 Starting...");

    // Initialize synchronization primitives
    xSensorDataMutex = xSemaphoreCreateMutex();
    xSDCardMutex = xSemaphoreCreateMutex();
    xTelemetrySemaphore = xSemaphoreCreateBinary();

    // Create tasks for both cores
    // Core 0 (main core) - Critical tasks
    xTaskCreateAffinitySet(
            Core0Task,
            "Core0Task",
            CORE0_STACK_SIZE,
            NULL,
            CORE0_PRIORITY,
            (1 << 0), // Run on core 0
            NULL
    );

    // Core 1 - Non-critical tasks
    xTaskCreateAffinitySet(
            Core1Task,
            "Core1Task",
            CORE1_STACK_SIZE,
            NULL,
            CORE1_PRIORITY,
            (1 << 1), // Run on core 1
            NULL
    );

    // Start the scheduler - This will not return
    vTaskStartScheduler();

    // If we get here, something went wrong with FreeRTOS
    Serial.println("ERROR: FreeRTOS scheduler failed to start!");
    while (1) {
        // Error loop
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

void loop() {
    // Empty - everything happens in FreeRTOS tasks
    // This function will never be called when FreeRTOS is running
}

// Core 0 main task - Handles critical flight operations
void Core0Task(void *pvParameters) {
    Serial.println("Core 0 task started");

    // Setup LED for status indication
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize I2C bus for sensors
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.begin();
    Wire.setClock(400000);  // 400kHz fast mode

    // Initialize SPI buses
    vspi.begin();
    hspi.begin();

    // Initialize barometric sensors
    BMP388Sensor* bmp388 = new BMP388Sensor(Wire);
    MPL3115A2Sensor* mpl3115a2 = new MPL3115A2Sensor(Wire);

    baroManager.addSensor(bmp388);
    baroManager.addSensor(mpl3115a2);

    // Initialize IMU sensors
    BMI088Sensor* bmi088 = new BMI088Sensor(Wire);
    ADXL375Sensor* adxl375 = new ADXL375Sensor(Wire);

    imuManager.addSensor(bmi088);
    imuManager.addSensor(adxl375);

    powerManager = new PowerManager(BATTERY_VOLTAGE_PIN, &storageManager);
    powerManager->setLowVoltageThreshold(3.5f);      // 3.5V for low battery warning
    powerManager->setCriticalVoltageThreshold(3.2f); // 3.2V for critical battery level
    powerManager->setBatteryFullVoltage(4.2f);       // 4.2V for fully charged LiPo
    powerManager->setBatteryEmptyVoltage(3.0f);      // 3.0V for empty LiPo

    if (!powerManager->begin()) {
        Serial.println("ERROR: Failed to initialize power manager!");

        // Log the error if storage is working
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SYSTEM, "Failed to initialize power manager");
        }
    }

    resourceMonitor = new ResourceMonitor(&storageManager);
    resourceMonitor->begin();

    // Initialize fault handler
    faultHandler = new FaultHandler(&storageManager, &stateMachine);
    faultHandler->begin();

    // Initialize performance optimizer
    performanceOptimizer = new PerformanceOptimizer(
            &storageManager,
            &stateMachine,
            resourceMonitor
    );
    performanceOptimizer->begin();

    // Initialize power controller
    powerController = new PowerController(
            powerManager,
            &baroManager,
            &imuManager,
            &gpsManager,
            loraSystem,
            &stateMachine
    );

    if (!powerController->begin()) {
        Serial.println("ERROR: Failed to initialize power controller!");

        // Log the error if storage is working
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SYSTEM, "Failed to initialize power controller");
        }
    }

    // Initialize diagnostic manager
    diagnosticManager = new DiagnosticManager(&storageManager);
    diagnosticManager->setVerboseLogging(true);

    diagnosticManager->addTest(new BarometricSensorTest(&baroManager));
    diagnosticManager->addTest(new IMUSensorTest(&imuManager));
    diagnosticManager->addTest(new GPSSensorTest(&gpsManager));
    diagnosticManager->addTest(new LoRaCommunicationTest(loraSystem));
    diagnosticManager->addTest(new StorageTest(&storageManager));
    diagnosticManager->addTest(new BatteryTest(powerManager));
    diagnosticManager->addTest(new SensorFusionTest(fusionSystem));

    preflightSystem = new PreflightCheckSystem(diagnosticManager, &storageManager);
    preflightSystem->begin();

    // Initialize GPS sensors
    Serial1.begin(9600);
    Serial2.begin(9600);

    L76KBGPSSensor* l76kb = new L76KBGPSSensor(Serial1, L76_STBY);
    ATGM336HGPSSensor* atgm336h = new ATGM336HGPSSensor(Serial2, ATGM_STBY);

    gpsManager.addSensor(l76kb, 0);  // Priority 0 (primary)
    gpsManager.addSensor(atgm336h, 1);  // Priority 1 (secondary)

    // Initialize LoRa
    loraSystem = new LoRaSystem(hspi, LORA_CS, LORA_RST, LORA_DIO0, -1, -1, &storageManager);

    // Initialize storage
    flashStorage = new FlashStorage(hspi, FLASH_CS);
    sdStorage = new SDStorage(vspi, SD_CS);

    storageManager.addStorage(flashStorage, true);  // Primary
    storageManager.addStorage(sdStorage, false);    // Secondary

    // Initialize everything
    bool initSuccess = true;

    initSuccess &= baroManager.begin();
    initSuccess &= imuManager.begin();
    initSuccess &= gpsManager.begin();

    initSuccess &= loraSystem->begin() == SensorStatus::OK;
    initSuccess &= storageManager.begin();

    Serial.println("Running initial diagnostics...");
    std::vector<TestResult> initialResults = diagnosticManager->runAllTests();

    int passCount = 0;
    for (const auto& result : initialResults) {
        if (result.passed) passCount++;
    }

    Serial.print("Initial diagnostics complete: ");
    Serial.print(passCount);
    Serial.print("/");
    Serial.print(initialResults.size());
    Serial.println(" tests passed");

    if (!initSuccess) {
        Serial.println("ERROR: Failed to initialize all subsystems!");

        // Log the error if storage is working
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SYSTEM, "Failed to initialize all subsystems");
        }
    } else {
        Serial.println("All subsystems initialized successfully");

        // Log success
        storageManager.logMessage(LogLevel::INFO, Subsystem::SYSTEM, "All subsystems initialized successfully");
    }

    // Initialize sensor fusion
    fusionSystem = new SensorFusionSystem(&baroManager, &imuManager, &storageManager);
    if (!fusionSystem->begin()) {
        Serial.println("ERROR: Failed to initialize sensor fusion system!");

        // Log the error if storage is working
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SENSORS, "Failed to initialize sensor fusion system");
        }
    }

    // Initialize and setup state machine
    stateMachine.begin(&baroManager, &imuManager, &gpsManager, loraSystem, &storageManager);
    StateHandlers::setupHandlers(stateMachine, &baroManager, &imuManager, &gpsManager, loraSystem, &storageManager, fusionSystem, diagnosticManager, preflightSystem, powerManager);

    // Signal that sensors are initialized
    sensorsInitialized = true;

    // Main task loop
    while (1) {
        // Update sensor fusion
        fusionSystem->update();

        resourceMonitor->update();
        performanceOptimizer->update();

        // Check for low memory condition and report fault if needed
        if (resourceMonitor->isMemoryLow()) {
            faultHandler->reportFault(
                    FaultType::MEMORY_ERROR,
                    FaultSeverity::WARNING,
                    "Low memory condition detected",
                    resourceMonitor->getFreeHeap()
            );
        }

        // Check for high CPU usage and report fault if needed
        if (resourceMonitor->isCpuHigh()) {
            faultHandler->reportFault(
                    FaultType::INTERNAL_ERROR,
                    FaultSeverity::WARNING,
                    "High CPU usage detected",
                    static_cast<uint32_t>(resourceMonitor->getCpuUsage() * 100)
            );
        }

        // Periodically log resource usage (every 5 minutes)
        static unsigned long lastResourceLogTime = 0;
        if (millis() - lastResourceLogTime > 300000) { // 5 minutes
            lastResourceLogTime = millis();
            resourceMonitor->logResourceUsage();
        }

        // Get fused data
        FusedFlightData fusedData = fusionSystem->getFusedData();

        // Use fused data for apogee detection
        if (fusedData.apogeeDetected && !apogeeDetectedFlag) {
            apogeeDetectedFlag = true;
            stateMachine.processEvent(RocketEvent::APOGEE_DETECTED);

            // Log apogee detection
            if (storageManager.isOperational()) {
                char message[50];
                snprintf(message, sizeof(message), "Apogee detected at %.1fm", fusedData.apogeeAltitude);
                storageManager.logMessage(LogLevel::INFO, Subsystem::STATE_MACHINE, message);
            }
        }

        powerManager->update();
        powerController->update();

        // Update state machine
        stateMachine.update();

        // Blink LED to indicate system is running
        static unsigned long lastLedTime = 0;
        if (millis() - lastLedTime >= 250) {  // 4Hz blink
            lastLedTime = millis();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }

        // Give other tasks a chance to run
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms cycle for critical tasks
    }
}

// Core 1 main task - Handles non-critical operations
void Core1Task(void *pvParameters) {
    Serial.println("Core 1 task started");

    // Wait for sensors to be initialized by Core 0
    while (!sensorsInitialized) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Signal that secondary systems are initialized
    storageInitialized = true;
    communicationInitialized = true;

    commandHandler = new CommandHandler(
            loraSystem,
            &storageManager,
            &stateMachine,
            powerManager,
            diagnosticManager,
            fusionSystem,
            &baroManager,
            &imuManager,
            &gpsManager
    );

    if (!commandHandler->begin()) {
        Serial.println("ERROR: Failed to initialize command handler!");

        // Log the error if storage is working
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::COMMUNICATION,
                                      "Failed to initialize command handler");
        }
    }

    // Main task loop
    while (1) {
        // Update command handler - this handles all LoRa command processing
        if (commandHandler) {
            commandHandler->update();
        }

        // Other non-critical tasks...

        // Give other tasks a chance to run
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms cycle for non-critical tasks
    }
}