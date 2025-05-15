//region Pruebas
////region Lora prueba
//#include <Arduino.h>
//#include <SPI.h>
//#include <LoRa.h>
//
//// RP2040 pin definitions - adjust if needed
//#define LORA_CS     9
//#define LORA_RST    12
//#define LORA_DIO0   13
//#define SPI1_MISO   8
//#define SPI1_MOSI   11
//#define SPI1_SCK    10
//#define LED_PIN     25
//
//// Message configuration
//#define TX_INTERVAL 200
//int counter = 0;
//
//void setup() {
//    // Initialize serial at high speed for debugging
//    Serial.begin(115200);
//    delay(3000);  // Time to open serial monitor
//    Serial.println("=== LoRa Simple Transmitter Test ===");
//
//    // Setup LED
//    pinMode(LED_PIN, OUTPUT);
//    digitalWrite(LED_PIN, LOW);
//
//    // Configure SPI for LoRa
//    SPI1.setRX(SPI1_MISO);
//    SPI1.setTX(SPI1_MOSI);
//    SPI1.setSCK(SPI1_SCK);
//    SPI1.begin();
//
//    // Configure LoRa module with clear error handling
//    Serial.println("Initializing LoRa module...");
//    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
//    LoRa.setSPI(SPI1);
//
//    // Reset the module first (important!)
//    pinMode(LORA_RST, OUTPUT);
//    digitalWrite(LORA_RST, LOW);
//    delay(20);
//    digitalWrite(LORA_RST, HIGH);
//    delay(150);
//
//    // Initialize with 868MHz frequency (EU standard)
//    // Use 915E6 for US or 433E6 for Asia
//    if (!LoRa.begin(868E6)) {
//        Serial.println("ERROR: LoRa initialization failed!");
//        // Blink rapidly to indicate failure
//        while (1) {
//            digitalWrite(LED_PIN, HIGH);
//            delay(100);
//            digitalWrite(LED_PIN, LOW);
//            delay(100);
//        }
//    }
//
//    // Use reliable, conservative settings
//    LoRa.setSpreadingFactor(9);      // Range 6-12, higher = more range but slower
//    LoRa.setSignalBandwidth(125E3);  // 125kHz bandwidth
//    LoRa.setCodingRate4(5);          // 4/5 coding rate
//    LoRa.setPreambleLength(8);       // Default preamble length
//    LoRa.setSyncWord(0x34);
//    LoRa.enableCrc();                // Enable CRC checking
//    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN); // Higher power
//
//    Serial.println("LoRa Transmitter initialized successfully!");
//
//    // Indicate success with three slow blinks
//    for (int i = 0; i < 3; i++) {
//        digitalWrite(LED_PIN, HIGH);
//        delay(200);
//        digitalWrite(LED_PIN, LOW);
//        delay(200);
//    }
//}
//
//void loop() {
//    Serial.print("Sending packet #");
//    Serial.print(counter);
//    Serial.print(": 'Mensaje de prueba para LoRa, 44 bytes'");
//
//    // Visual indicator
//    digitalWrite(LED_PIN, HIGH);
//
//    // Start LoRa packet
////    LoRa.idle();  // Ensure idle state before transmission
//
//    if (!LoRa.beginPacket()) {
//        Serial.println(" - ERROR: Could not start packet");
//        digitalWrite(LED_PIN, LOW);
//        delay(100);
//        digitalWrite(LED_PIN, HIGH);
//        delay(100);
//        digitalWrite(LED_PIN, LOW);
//        delay(TX_INTERVAL);
//        return;
//    }
//
//    // Write message content
//    LoRa.print("Mensaje de prueba para LoRa de 44 bytes#");
//    LoRa.print(counter);
//
//    // End packet with explicit confirmation
//    bool sent = LoRa.endPacket(true);
//
//    if (sent) {
//        Serial.println(" - SUCCESS");
//    } else {
//        Serial.println(" - ERROR: Transmission failed");
//    }
//
//    // Turn off indicator LED
//    digitalWrite(LED_PIN, LOW);
//
//    // Increment counter
//    counter++;
//
//    // Wait for next transmission
//    delay(TX_INTERVAL);
//}
////endregion Lora prueba
//endregion

//region MainCode
//region to close
#include <Arduino.h>
#include "Config.h"
#include "PinDefinitions.h"

#include "../lib/Diagnostics/DiagnosticManager.h"
#include "../lib/Diagnostics/SpecificTests.h"
#include "../lib/Diagnostics/PreflightCheck.h"

#include "../lib/Optimization/ResourceMonitor.h"
#include "../lib/Optimization/FaultHandler.h"

// Include HAL components
#include "../lib/HAL/BarometricSensors/BarometricSensorManager.h"
#include "../lib/HAL/BarometricSensors/BMP388Sensor.h"
#include "../lib/HAL/BarometricSensors/MPL3115A2Sensor.h"

#include "../lib/HAL/IMUSensors/IMUSensorManager.h"
#include "../lib/HAL/IMUSensors/BMI088Sensor.h"
#include "../lib/HAL/IMUSensors/ADXL375Sensor.h"

#include "../lib/HAL/TemperatureSensors/DS18B20Sensor.h"
#include "../lib/HAL/TemperatureSensors/TemperatureSensorManager.h"

#include "../lib/HAL/GPSSensors/GPSSensorManager.h"
#include "../lib/HAL/GPSSensors/L76KBGPSSensor.h"
#include "../lib/HAL/GPSSensors/ATGM336HGPSSensor.h"

#include "../lib/PowerManagement/PowerManager.h"
#include "../lib/PowerManagement/PowerController.h"

#include "../lib/HAL/CommunicationSystems/LoRaSystem.h"
#include "../lib/HAL/CommunicationSystems/TelemetrySerializer.h"

#include "../lib/HAL/StorageSystems/StorageManager.h"
#include "../lib/HAL/StorageSystems/FlashStorage.h"
#include "../lib/HAL/StorageSystems/SDStorage.h"

#include "../lib/Communication/RocketProtocol.h"
#include "../lib/Communication/CommandHandler.h"

// Include State Machine
#include "../lib/StateMachine/StateMachine.h"
#include "../lib/StateMachine/StateHandlers.h"
#include "SensorConfig.h"

// Status flags
volatile bool apogeeDetectedFlag = false;

// Sensor managers
BarometricSensorManager baroManager;
IMUSensorManager imuManager;
GPSSensorManager gpsManager;
TemperatureSensorManager temperatureManager;

// Optimization components
ResourceMonitor* resourceMonitor;
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

// State machine
StateMachine stateMachine;

DataIntegrationManager* dataManager;

// Sensor instances
MPL3115A2Sensor mpl3115Sensor = MPL3115A2Sensor(Wire1, MPL_ADDR);
BMP388Sensor bmp388Sensor = BMP388Sensor(Wire1, BMP_ADDR);

// Timing control variables
unsigned long lastPerformanceCheckTime = 0;
unsigned long lastResourceLogTime = 0;

void initializeAllSystems() {
    Serial.println("Adding barometric sensors...");
    baroManager.addSensor(&bmp388Sensor, 0);  // Primary sensor (priority 0)
    baroManager.addSensor(&mpl3115Sensor, 1);  // Secondary sensor (priority 1)
    Serial.println("Barometric sensors added");

    delay(1000);  // Wait for sensors to stabilize

    Serial.println("Adding IMU sensors...");
    BMI088Sensor* bmi088 = new BMI088Sensor(Wire1, BMIO_GYR_ADDR, BMIO_ACCEL_ADDR);
    ADXL375Sensor* adxl375 = new ADXL375Sensor(Wire1, AXL_ADDR);
    imuManager.addSensor(bmi088, 0);  // Primary sensor (priority 0)
    imuManager.addSensor(adxl375, 1);  // Secondary sensor (priority 1)
    Serial.println("IMU sensors added");

    delay(1000);  // Wait for sensors to stabilize

    Serial.println("Adding temperature sensors...");
    DS18B20Sensor* ds18b20 = new DS18B20Sensor(DS18B20_PIN);
    ds18b20->setAsyncResolution(9);
    temperatureManager.addSensor(ds18b20);
    Serial.println("Temperature sensors added");

    delay(1000);  // Wait for sensors to stabilize

    Serial.println("Adding GPS sensors...");
    L76KBGPSSensor* l76kb = new L76KBGPSSensor(Serial1, L76_STBY);
    ATGM336HGPSSensor* atgm336h = new ATGM336HGPSSensor(Serial2, ATGM_STBY);
    //TODO
//    gpsManager.addSensor(l76kb, 0);  // Priority 0 (primary)
//    gpsManager.addSensor(atgm336h, 1);  // Priority 1 (secondary)
    Serial .println("GPS sensors added");

    delay(1000);  // Wait for sensors to stabilize

    Serial.println("Initializing LoRaSystem...");
    loraSystem = new LoRaSystem(SPI1, LORA_CS, LORA_RST, LORA_DIO0, &storageManager);
    loraSystem->setNodeId(ROCKET_ID);
    loraSystem->setDestinationId(GROUND_STATION_ID);
    Serial.println("LoRaSystem initialized");

    delay(1000);  // Wait for LoRa system to stabilize

    Serial.println("Adding storage systems...");
    flashStorage = new FlashStorage(SPI, FLASH_CS);
    sdStorage = new SDStorage(SPI1, SD_CS);
    storageManager.addStorage(flashStorage, true);  // Primary
    storageManager.addStorage(sdStorage, false);    // Secondary
    Serial.println("Storage systems added");

    delay(1000);  // Wait for storage systems to stabilize

    Serial.println("Initializing power manager...");
    powerManager = new PowerManager(BATTERY_VOLTAGE_PIN, &storageManager);
    powerManager->setLowVoltageThreshold(3.5f);      // 3.5V for low battery warning
    powerManager->setCriticalVoltageThreshold(3.2f); // 3.2V for critical battery level
    powerManager->setBatteryFullVoltage(4.2f);       // 4.2V for fully charged LiPo
    powerManager->setBatteryEmptyVoltage(3.0f);      // 3.0V for empty LiPo

    if (!powerManager->begin()) {
        Serial.println("ERROR: Failed to initialize power manager!");
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SYSTEM, "Failed to initialize power manager");
        }
    }
    Serial.println("Power manager initialized");

    delay(1000);  // Wait for power manager to stabilize

    Serial.println("Initializing resource monitor...");
    resourceMonitor = new ResourceMonitor(&storageManager);
    resourceMonitor->begin();
    Serial.println("Resource monitor initialized");

    delay(1000);  // Wait for resource monitor to stabilize

    Serial.println("Initializing fault handler...");
    faultHandler = new FaultHandler(&storageManager, &stateMachine);
    faultHandler->begin();
    Serial.println("Fault handler initialized");

    delay(1000);  // Wait for fault handler to stabilize

    Serial.println("Initializing power controller...");
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
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SYSTEM, "Failed to initialize power controller");
        }
    }
    Serial.println("Power controller initialized");

    delay(1000);  // Wait for power controller to stabilize

    // Initialize each subsystem
    bool initSuccess = true;

    Serial.println("Initializing barometric sensors...");
    initSuccess &= baroManager.begin();
    Serial.println("Barometric sensors initialized");

    delay(1000);  // Wait for barometric sensors to stabilize

    Serial.println("Initializing IMU sensors...");
    initSuccess &= imuManager.begin();
    Serial.println("IMU sensors initialized");

    delay(1000);  // Wait for IMU sensors to stabilize

    Serial.println("Initializing loraSystem...");
    initSuccess &= loraSystem->begin() == SensorStatus::OK;
    Serial.println("LoRa system initialized");

    delay(1000);  // Wait for LoRa system to stabilize

    Serial.println("Initializing temperature sensors...");
    initSuccess &= temperatureManager.begin();
    Serial.println("Temperature sensors initialized");

    delay(1000);  // Wait for temperature sensors to stabilize

    Serial.println("Initializing storage systems...");
    initSuccess &= storageManager.begin();
    Serial.println("Storage systems initialized");

    delay(1000);  // Wait for storage systems to stabilize

    Serial.println("Initializing GPS sensors...");
    initSuccess &= gpsManager.begin();
    Serial.println("GPS sensors initialized");

    delay(1000);  // Wait for GPS sensors to stabilize

    Serial.println("Initializing data integration manager...");
    dataManager = new DataIntegrationManager(&baroManager, &imuManager, &gpsManager, &temperatureManager, powerManager);
    if (!dataManager->begin()) {
        Serial.println("ERROR: Failed to initialize data integration manager!");
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SENSORS,
                                      "Failed to initialize data integration manager");
        }
    }
    Serial.println("Data integration manager initialized");

    delay(1000);  // Wait for data integration manager to stabilize

    Serial.println("Initializing DiagnosticManager...");
    diagnosticManager = new DiagnosticManager(&storageManager);
    diagnosticManager->setVerboseLogging(true);
    Serial.println("DiagnosticManager initialized");

    delay(1000);  // Wait for diagnostic manager to stabilize

    Serial.println("Adding diagnostic tests...");
    diagnosticManager->addTest(new BarometricSensorTest(&baroManager));
    diagnosticManager->addTest(new IMUSensorTest(&imuManager));
    diagnosticManager->addTest(new GPSSensorTest(&gpsManager));
    diagnosticManager->addTest(new LoRaCommunicationTest(loraSystem));
    diagnosticManager->addTest(new StorageTest(&storageManager));
    diagnosticManager->addTest(new BatteryTest(powerManager));
    diagnosticManager->addTest(new TemperatureSensorTest(&temperatureManager));
    Serial.println("Diagnostic tests added");

    delay(1000);  // Wait for diagnostic tests to stabilize

    Serial.println("Initializing PreflightCheckSystem...");
    preflightSystem = new PreflightCheckSystem(diagnosticManager, &storageManager);
    preflightSystem->begin();
    Serial.println("PreflightCheckSystem initialized");

    delay(1000);  // Wait for preflight check system to stabilize

    Serial.println("Running initial diagnostics...");
    // Double beep to indicate diagnostics start
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);

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

        // Print all the errors
        for (const auto& result : initialResults) {
            if (!result.passed) {
                Serial.print("Test: ");
                Serial.print(result.testName);
                Serial.print(" - Error: ");
                Serial.println(result.errorMessage);
            }
        }
    } else {
        Serial.println("All subsystems initialized successfully");
        // Log success
        storageManager.logMessage(LogLevel::INFO, Subsystem::SYSTEM, "All subsystems initialized successfully");
    }

    // Initialize and setup state machine
    Serial.println("Initializing state machine...");
    stateMachine.begin(&baroManager, &imuManager, &gpsManager, loraSystem, &storageManager, dataManager);
    StateHandlers::setupHandlers(stateMachine, dataManager, loraSystem, &storageManager, diagnosticManager, preflightSystem, powerManager);
    Serial.println("State machine initialized");

    delay(1000);  // Wait for state machine to stabilize

    // Long beep to indicate initialization complete
//    digitalWrite(BUZZER_PIN, HIGH);
//    delay(1000);
//    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Initializing command handler...");
    commandHandler = new CommandHandler(
            loraSystem,
            &storageManager,
            &stateMachine,
            powerManager,
            diagnosticManager,
            dataManager,
            &baroManager,
            &imuManager,
            &gpsManager
    );

    if (!commandHandler->begin()) {
        Serial.println("ERROR: Failed to initialize command handler!");
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::COMMUNICATION,
                                      "Failed to initialize command handler");
        }
    }
    Serial.println("Command handler initialized");

    delay(1000);  // Wait for command handler to stabilize

    // Initialize timing variables
    lastPerformanceCheckTime = millis();
    lastResourceLogTime = millis();
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Wait for serial port to connect or timeout of 5 seconds
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 5000)) {
        delay(10);
    }

    delay(2000);

    // Initialize pins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Signal startup
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Initializing rocket control system...");

    // Initialize I2C, SPI, and UART
    Wire1.setSDA(I2C1_SDA);
    Wire1.setSCL(I2C1_SCL);
    Wire1.begin();

    delay(1000); // Allow I2C to stabilize

    SPI1.setMISO(SPI1_MISO);
    SPI1.setMOSI(SPI1_MOSI);
    SPI1.setSCK(SPI1_SCK);
    SPI1.begin();

    delay(1000); // Allow SPI to stabilize

    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    Serial1.begin(9600);
    Serial2.begin(9600);

    delay(1000); // Allow UART to stabilize

    // Initialize all systems
    initializeAllSystems();

    Serial.println("Setup complete, entering main loop");
}

void loop() {
    // *** CRITICAL OPERATIONS ***

    // Update resource monitoring and optimization
    resourceMonitor->update();

    // Check system health periodically
    unsigned long currentTime = millis();
    if (currentTime - lastPerformanceCheckTime >= 300000) { // Every 10 seconds
        lastPerformanceCheckTime = currentTime;

        // Check for low memory condition
        if (resourceMonitor->isMemoryLow()) {
            faultHandler->reportFault(
                    FaultType::MEMORY_ERROR,
                    FaultSeverity::WARNING,
                    "Low memory condition detected",
                    resourceMonitor->getFreeHeap()
            );
        }

        // Check for high CPU usage
        if (resourceMonitor->isCpuHigh()) {
            faultHandler->reportFault(
                    FaultType::INTERNAL_ERROR,
                    FaultSeverity::WARNING,
                    "High CPU usage detected",
                    static_cast<uint32_t>(resourceMonitor->getCpuUsage() * 100)
            );
        }
    }
//
//    // Log resource usage periodically
    if (currentTime - lastResourceLogTime > 300000) { // Every 5 minutes
        lastResourceLogTime = currentTime;
        resourceMonitor->logResourceUsage();
    }
//
//    // Update power management
//    powerManager->update();
//    powerController->update();

    // Update state machine
    stateMachine.update();

    // *** NON-CRITICAL OPERATIONS ***

    // Process commands
//    if (commandHandler) {
//        commandHandler->update();
//    }

    // Brief delay to prevent hammering the CPU
//    delay(10);
}
//endregion
//endregion