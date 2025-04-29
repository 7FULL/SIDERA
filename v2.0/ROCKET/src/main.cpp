///**
// * Rocket Control System - RP2040 Version
// * Main entry point
// *
// * This file initializes the hardware, sets up FreeRTOS tasks
// * and starts the scheduler on both cores.
// */
//
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

MPL3115A2Sensor mpl3115Sensor = MPL3115A2Sensor(Wire1, MPL_ADDR);
BMP388Sensor bmp388Sensor = BMP388Sensor(Wire1, BMP_ADDR);

//Adafruit_MPL3115A2  mpl3115 = Adafruit_MPL3115A2();
//Adafruit_BMP3XX bmp388 = Adafruit_BMP3XX();

void initializeAllSystems(){
    Serial.println("Core 0 task started");

    Serial.println("Adding barometric sensors...");

    baroManager.addSensor(&mpl3115Sensor);
    baroManager.addSensor(&bmp388Sensor);

    Serial.println("Barometric sensors added");

    // Initialize IMU sensors
    BMI088Sensor* bmi088 = new BMI088Sensor(Wire1, BMIO_GYR_ADDR, BMIO_ACCEL_ADDR);
    ADXL375Sensor* adxl375 = new ADXL375Sensor(Wire1, AXL_ADDR);

    digitalWrite(LED_BLUE, HIGH);
    Serial.println("Adding IMU sensors...");

    imuManager.addSensor(bmi088);
    imuManager.addSensor(adxl375);

    Serial.println("IMU sensors added");
    Serial.println("Initializing power manager...");

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

    Serial.println("Power manager initialized");
    Serial.println("Initializing communication system...");

    resourceMonitor = new ResourceMonitor(&storageManager);
    resourceMonitor->begin();

    Serial.println("Resource monitor initialized");
    Serial.println("Initializing fault handler...");

    // Initialize fault handler
    faultHandler = new FaultHandler(&storageManager, &stateMachine);
    faultHandler->begin();

    Serial.println("Fault handler initialized");
    Serial.println("Initializing performance optimizer...");

    // Initialize performance optimizer
    performanceOptimizer = new PerformanceOptimizer(
            &storageManager,
            &stateMachine,
            resourceMonitor
    );
    performanceOptimizer->begin();

    Serial.println("Performance optimizer initialized");
    Serial.println("Initializing power controller...");

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

    Serial.println("Power controller initialized");
    Serial.println("Initializing gps...");

    // Initialize GPS sensors
//    Serial1.setRX(L76_RX);
//    Serial1.setTX(L76_TX);

//    Serial2.setRX(ATGM_RX);
//    Serial2.setTX(ATGM_TX);

    Serial1.customSetPinsUart0();
    Serial2.customSetPinsUart1();

    Serial1.begin(9600);
    Serial2.begin(9600);

    L76KBGPSSensor* l76kb = new L76KBGPSSensor(Serial1, L76_STBY);
    ATGM336HGPSSensor* atgm336h = new ATGM336HGPSSensor(Serial2, ATGM_STBY);

    Serial.println("Adding GPS sensors...");

    gpsManager.addSensor(l76kb, 0);  // Priority 0 (primary)
    gpsManager.addSensor(atgm336h, 1);  // Priority 1 (secondary)

    Serial.println("GPS sensors added");
    Serial.println("Initializing LoRaSystem...");

    // Initialize LoRa
    loraSystem = new LoRaSystem(SPI1, LORA_CS, LORA_RST, LORA_DIO0, &storageManager);

    Serial.println("LoRaSystem initialized");
    Serial.println("Initializing storage systems...");

    // Initialize storage
    flashStorage = new FlashStorage(SPI1, FLASH_CS);
    sdStorage = new SDStorage(SPI, SD_CS);

    Serial.println("Adding storage systems...");

    storageManager.addStorage(flashStorage, true);  // Primary
    storageManager.addStorage(sdStorage, false);    // Secondary

    Serial.println("Storage systems added");

    // Initialize everything
    bool initSuccess = true;

    Serial.println("Initializing all managers...");

//    if (bmp388->begin() != SensorStatus::OK) {
//        Serial.println("ERROR: Failed to initialize BMP388 sensor!");
//    }else{
//        Serial.println("BMP388 sensor initialized");
//    }

    Serial.println("Initializing barometric sensors...");
    initSuccess &= baroManager.begin();
    Serial.println("Barometric sensors initialized");

    Serial.println("Initializing IMU sensors...");
    initSuccess &= imuManager.begin();
    Serial.println("IMU sensors initialized");

    Serial.println("Initializing GPS sensors...");
    initSuccess &= gpsManager.begin();
    Serial.println("GPS sensors initialized");

    Serial.println("Initializing loraSystem...");
    initSuccess &= loraSystem->begin() == SensorStatus::OK;
    Serial.println("LoRa system initialized");

    Serial.println("Initializing storage systems...");
    initSuccess &= storageManager.begin();
    Serial.println("Storage systems initialized");

    // Initialize sensor fusion
    fusionSystem = new SensorFusionSystem(&baroManager, &imuManager, &storageManager);
    if (!fusionSystem->begin()) {
        Serial.println("ERROR: Failed to initialize sensor fusion system!");

        // Log the error if storage is working
        if (storageManager.isOperational()) {
            storageManager.logMessage(LogLevel::ERROR, Subsystem::SENSORS, "Failed to initialize sensor fusion system");
        }
    }

    Serial.println("Initializing DiagnosticManager...");

    // Initialize diagnostic manager
    diagnosticManager = new DiagnosticManager(&storageManager);
    diagnosticManager->setVerboseLogging(true);

    Serial.println("DiagnosticManager initialized");
    Serial.println("Adding diagnostic tests...");

    diagnosticManager->addTest(new BarometricSensorTest(&baroManager));
    diagnosticManager->addTest(new IMUSensorTest(&imuManager));
    diagnosticManager->addTest(new GPSSensorTest(&gpsManager));
    diagnosticManager->addTest(new LoRaCommunicationTest(loraSystem));
    diagnosticManager->addTest(new StorageTest(&storageManager));
    diagnosticManager->addTest(new BatteryTest(powerManager));
    diagnosticManager->addTest(new SensorFusionTest(fusionSystem));

    Serial.println("Diagnostic tests added");
    Serial.println("Initializing PreflightCheckSystem...");

    preflightSystem = new PreflightCheckSystem(diagnosticManager, &storageManager);
    preflightSystem->begin();

    Serial.println("PreflightCheckSystem initialized");

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

        //Print all the errors
        for (const auto& result : initialResults) {
            if (!result.passed) {
                Serial.print("Test: ");
                Serial.print(result.testName);
                Serial.print(" - Error: ");
                Serial.println(result.errorMessage);
            }
        }

        while (1) {
            // Error loop
            digitalWrite(LED_RED, HIGH);
            delay(100);
            digitalWrite(LED_RED, LOW);
            delay(100);
        }
    } else {
        Serial.println("All subsystems initialized successfully");

        // Log success
        storageManager.logMessage(LogLevel::INFO, Subsystem::SYSTEM, "All subsystems initialized successfully");
    }

    // Initialize and setup state machine
    stateMachine.begin(&baroManager, &imuManager, &gpsManager, loraSystem, &storageManager);
    StateHandlers::setupHandlers(stateMachine, &baroManager, &imuManager, &gpsManager, loraSystem, &storageManager, fusionSystem, diagnosticManager, preflightSystem, powerManager);

    // Signal that sensors are initialized
    sensorsInitialized = true;
}

void setup() {
    delay(1000);
    // Initialize serial communication
    Serial.begin(115200);

    while (!Serial) {
        delay(10); // Wait for serial port to connect
    }

    Serial.println("Initializing system...");

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    Wire1.setSDA(I2C1_SDA);
    Wire1.setSCL(I2C1_SCL);
    Wire1.begin();

    // Initialize all systems
    initializeAllSystems();

//    mpl3115Sensor.begin();
//    bmp388Sensor.begin();

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
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        delay(100);
    }
}

void loop() {
    //Ponemos el led rojo a parpadear
//    digitalWrite(LED_RED, !digitalRead(LED_RED));
//    delay(100);
}

// Core 0 main task - Handles critical flight operations
void Core0Task(void *pvParameters) {
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

    return;
    Serial.println("Core 1 task started");

    // Wait for sensors to be initialized by Core 0
    while (!sensorsInitialized) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("Core 1: Sensors initialized");

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

    Serial.println("Core 1: Command handler initialized");

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

//void setup() {
//    Serial.begin(115200);
//    // Espera activa hasta que el monitor serie esté abierto
//    while (!Serial) {
//        delay(10);
//    }
//
//    // Initialize I2C bus for sensors
//    Wire1.setSDA(I2C1_SDA);
//    Wire1.setSCL(I2C1_SCL);
//
//    Serial.println("Setting up I2C pins...");
//
//    Wire1.begin();
//    Wire1.setClock(400000);  // 400kHz fast mode
//
//    // Initialize barometric sensors
//    BMP388Sensor* bmp388 = new BMP388Sensor(Wire1, BMP_ADDR);
//
//    bmp388->begin();
//
//    Serial.println("Barometric sensor initialized");
//}
//
//void loop() {
//
//}

//#include <Arduino.h>
//#include <SPI.h>
//#include <SdFat.h>
//
//// SD card pins from schematic
//#define SD_CS_PIN 16
//
//// Create SD card object
//SdFat sd;
//
//void setup() {
//    // Initialize serial for debugging
//    Serial.begin(115200);
//    while (!Serial) {
//        delay(10); // Wait for serial port to connect
//    }
//    Serial.println("SD Card Test");
//
//    // Configure SPI pins if needed
////    SPI1.setRX(SD_MISO_PIN);
////    SPI1.setTX(SD_MOSI_PIN);
////    SPI1.setSCK(SD_SCK_PIN);
//
//    SPI1.setCustomPins();
//    SPI1.begin();
//
//    // Initialize SD card
//    Serial.print("Initializing SD card...");
//
//    if (!sd.begin(SdSpiConfig(SD_CS_PIN, 1, SD_SCK_MHZ(25), &SPI1))) {
//        Serial.println("initialization failed!");
//        Serial.println("Things to check:");
//        Serial.println("* is a card inserted?");
//        Serial.println("* is your wiring correct?");
//        Serial.println("* did you change the chipSelect pin to match your shield or module?");
//        while (1); // Stop here
//    }
//
//    Serial.println("initialization done.");
//
//    // Print the card type
//    Serial.print("Card type: ");
//    switch (sd.card()->type()) {
//        case SD_CARD_TYPE_SD1:
//            Serial.println("SD1");
//            break;
//        case SD_CARD_TYPE_SD2:
//            Serial.println("SD2");
//            break;
//        case SD_CARD_TYPE_SDHC:
//            Serial.println("SDHC");
//            break;
//        default:
//            Serial.println("Unknown");
//    }
//
//    // List files in root directory
//    Serial.println("Files found in root directory:");
//    SdFile root;
//    SdFile file;
//
//    root.openRoot(sd.vol());
//    while (file.openNext(&root, O_READ)) {
//        char fileName[13];
//        file.getName(fileName, sizeof(fileName));
//        Serial.print("  ");
//        Serial.print(fileName);
//        Serial.print("  ");
//        Serial.print(file.fileSize());
//        Serial.println(" bytes");
//        file.close();
//    }
//
//    // Create a new file
//    Serial.println("Creating test.txt...");
//    SdFile testFile;
//    if (!testFile.open("test.txt", O_WRITE | O_CREAT | O_TRUNC)) {
//        Serial.println("Error opening test.txt");
//        while (1);
//    }
//
//    // Write something to the file
//    testFile.println("Hello from Raspberry Pi Pico!");
//    testFile.println("If you can read this, your SD card is working!");
//    testFile.close();
//    Serial.println("Done writing to test.txt");
//
//    // Re-open the file to read and verify
//    if (!testFile.open("test.txt", O_READ)) {
//        Serial.println("Error re-opening test.txt");
//        while (1);
//    }
//
//    Serial.println("Reading from test.txt:");
//    char line[100];
//    int i = 0;
//    while (testFile.available()) {
//        line[i] = testFile.read();
//        if (line[i] == '\n' || i >= sizeof(line) - 1) {
//            line[i + 1] = 0; // Null terminate
//            Serial.print(line);
//            i = 0;
//        } else {
//            i++;
//        }
//    }
//    testFile.close();
//
//    Serial.println("\nSD Card test completed successfully!");
//}
//
//void loop() {
//    // Nothing to do in loop
//    delay(1000);
//}