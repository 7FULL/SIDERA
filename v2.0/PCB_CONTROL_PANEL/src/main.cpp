#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <SdFat.h>

// Pin Definitions (same as rocket for hardware compatibility)
#define I2C1_SDA 2
#define I2C1_SCL 3

#define SPI1_MISO 8
#define SPI1_MOSI 11
#define SPI1_SCK 10
#define LORA_CS 9
#define LORA_RST 12
#define LORA_DIO0 13

#define SPI0_MISO 16
#define SPI0_MOSI 19
#define SPI0_SCK 18
#define SD_CS 16

#define LED_RED 21
#define LED_BLUE 22

// LoRa settings
#define LORA_FREQUENCY 868E6  // Use 915E6 for US
#define LORA_SPREADING_FACTOR 12
#define LORA_BANDWIDTH 62.5E3
#define LORA_CODING_RATE 8
#define LORA_TX_POWER 20

// Node IDs
#define GROUND_STATION_ID 0x01
#define ROCKET_ID 0x02

// Protocol definitions
// These must match the rocket's protocol definitions
enum class CommandCode : uint8_t {
    PING = 0x01,
    GET_STATUS = 0x02,
    WAKE_UP_COMMAND = 0x03,
    ABORT_COMMAND = 0x04,
    CALIBRATE_SENSORS = 0x05,
    RUN_DIAGNOSTICS = 0x06,
};

enum class ResponseCode : uint8_t {
    ACK = 0x01,
    NACK = 0x02,
    TELEMETRY_DATA = 0x10,
    STATUS_DATA = 0x11,
    PARAMETER_VALUE = 0x20,
    DIAGNOSTIC_RESULT = 0x30,
    EVENT_NOTIFICATION = 0x40,
    ERROR_MESSAGE = 0xE0
};

enum class RocketStatusCode : uint8_t {
    INIT,               // Initial state during boot
    GROUND_IDLE,        // On ground, waiting for commands
    READY,              // Ready for launch, all systems go
    POWERED_FLIGHT,     // Engine burning, accelerating
    COASTING,           // Engine off, still ascending
    APOGEE,             // At highest point
    DESCENT,            // Falling, no parachute
    PARACHUTE_DESCENT,  // Falling with parachute deployed
    LANDED,             // On ground after flight
    ERROR               // Error condition
};


// SD card for logging
SdFat sd;
FatFile logFile;
char logFileName[20];
uint32_t logCounter = 0;

// Telemetry packet structure
struct TelemetryPacket {
    uint32_t timestamp;            // Timestamp in milliseconds
    uint8_t rocketState;           // Current state of the rocket
    float altitude;                // Altitude in meters
    float verticalSpeed;           // Vertical speed in m/s
    float acceleration;            // Total acceleration in m/s²
    float temperature;             // Temperature in °C
    float pressure;                // Pressure in hPa
    float batteryVoltage;          // Battery voltage in V
    uint8_t gpsSatellites;         // Number of GPS satellites in use
    float gpsLatitude;             // GPS latitude in degrees
    float gpsLongitude;            // GPS longitude in degrees
    float gpsAltitude;             // GPS altitude in meters
    uint8_t sensorStatus;          // Bit field of sensor statuses
    uint8_t flags;                 // Bit field of flags (parachute deployed, etc.)
};

// Status variables
TelemetryPacket lastTelemetry = {0};
uint8_t rocketStatus = 0;
unsigned long lastTelemetryTime = 0;
unsigned long packetCounter = 0;
uint16_t sequenceNumber = 0;
bool rocketConnectionActive = false;
unsigned long lastPingTime = 0;
unsigned long lastPingResponseTime = 0;
int lastRssi = 0;
float lastSnr = 0.0f;

// Function prototypes
void initLoRa();
void initSD();
void handleSerialCommands();
void processLoRaPacket(int packetSize);
void sendCommand(CommandCode code, uint8_t* payload = nullptr, size_t length = 0);
void printTelemetry();
void logTelemetry();
void updateLeds();

void setup() {
    // Initialize serial for debugging and user interface
    Serial.begin(115200);
    while (!Serial && millis() < 10000);

    delay(1000); // Wait for serial to stabilize

    Serial.println("\n\n=========================================");
    Serial.println("Rocket Ground Station Control Panel");
    Serial.println("=========================================");

    // Setup pins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // Flash LEDs to indicate startup
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_BLUE, LOW);
        delay(100);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, HIGH);
        delay(100);
    }
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);

    // Initialize I2C
    Wire1.setSDA(I2C1_SDA);
    Wire1.setSCL(I2C1_SCL);
    Wire1.begin();

    delay(1000); // Wait for I2C to stabilize

    // Initialize SPI
    SPI1.setMISO(SPI1_MISO);
    SPI1.setMOSI(SPI1_MOSI);
    SPI1.setSCK(SPI1_SCK);
    SPI1.begin();

    delay(1000); // Wait for SPI to stabilize

    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    // Initialize LoRa and SD
    initLoRa();
    delay(1000); // Wait for LoRa to stabilize
    initSD();
}

void loop() {
    // Process any incoming LoRa packets
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        processLoRaPacket(packetSize);
    }

    // Handle any serial commands from the user
    handleSerialCommands();

    // Check connection status
//    checkConnection();

    // Update status LEDs
    updateLeds();

    // Short delay to avoid hammering the CPU
    delay(10);
}

void initLoRa() {
    Serial.print("Initializing LoRa... ");

    Serial.println("LoRa: Performing hardware reset...");
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(1000);
    digitalWrite(LORA_RST, HIGH);
    delay(1000);// Give the module more time to stabilize

    Serial.println("LoRa: Resetting SPI...");
    SPI1.end();
    delay(1000);
    SPI1.begin();
    delay(1000);

    // Configure LoRa
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    LoRa.setSPI(SPI1);

    // Initialize LoRa with multiple retries
    bool success = false;
    for (int attempt = 0; attempt < 5; attempt++) {
        if (LoRa.begin(LORA_FREQUENCY)) {
            success = true;
            break;
        }
        Serial.print(".");
        delay(500);
    }

    if (!success) {
        Serial.println("FAILED!");
        while (1) {
            digitalWrite(LED_RED, HIGH);
            delay(100);
            digitalWrite(LED_RED, LOW);
            delay(100);
        }
    }

    // Configure LoRa parameters
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSyncWord(0x34);  // Match the rocket's sync word
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();

    LoRa.onReceive(processLoRaPacket);

    // Start listening for packets
    LoRa.receive();

    Serial.println("SUCCESS!");
}

void initSD() {
    Serial.print("Initializing SD card... ");

    if (!sd.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(25), &SPI1))) {
        Serial.println("FAILED!");
        Serial.println("WARNING: SD logging will be disabled");
        return;
    }

    int fileCount = 0;
    FatFile dir;
    dir.open("/");
    FatFile file;
    while (file.openNext(&dir, O_RDONLY)) {
        char name[32];
        file.getName(name, sizeof(name));
        if (strstr(name, "LOG_") == name && strstr(name, ".CSV")) {
            fileCount++;
        }
        file.close();
    }
    dir.close();

    // Crear el archivo con el número correspondiente
    sprintf(logFileName, "LOG_%d.CSV", fileCount);
    if (!logFile.open(logFileName, O_WRITE | O_CREAT)) {
        Serial.println("Error creating log file");
        return;
    }

    // Escribir cabecera
    logFile.write("Timestamp,RocketState,Altitude,VertSpeed,Accel,Temp,Pressure,Battery,Sats,Lat,Lon,RSSI,SNR");
    logFile.flush();

    Serial.print("SUCCESS! Logging to ");
    Serial.println(logFileName);
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();

        // Process command
        switch (cmd) {
            case 'p':
                // Ping command
//                Serial.println("Sending PING command...");
                sendCommand(CommandCode::PING);
                break;
            case 'l':
                // Launch command
//                Serial.println("WARNING: SENDING LAUNCH COMMAND!");
//                Serial.println("Are you sure? (y/n)");
                sendCommand(CommandCode::WAKE_UP_COMMAND);
                break;

            case 'x':
                // Abort command
//                Serial.println("Sending ABORT command...");
                sendCommand(CommandCode::ABORT_COMMAND);
                break;

            case 'c':
                // Calibrate sensors
//                Serial.println("Calibrating sensors...");
                sendCommand(CommandCode::CALIBRATE_SENSORS);
                break;

            case 'r':
                // Run diagnostics
//                Serial.println("Running diagnostics...");
                sendCommand(CommandCode::RUN_DIAGNOSTICS);
                break;

            default:
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                break;
        }

        // Clear any remaining characters
        while (Serial.available()) {
            Serial.read();
        }
    }
}

// Helper function to read float from LoRa packet
float readFloat() {
    union {
        float f;
        uint8_t bytes[4];
    } converter;

    for (int i = 0; i < 4 && LoRa.available(); i++) {
        converter.bytes[i] = LoRa.read();
    }

    return converter.f;
}

// Helper function to read uint32_t from LoRa packet
uint32_t readUint32() {
    uint32_t value = 0;
    for (int i = 0; i < 4 && LoRa.available(); i++) {
        value |= (uint32_t)LoRa.read() << (i * 8);
    }
    return value;
}

void processLoRaPacket(int packetSize) {
//    Serial.print("Received packet of size: ");
//    Serial.println(packetSize);

    // Capture RSSI and SNR
    lastRssi = LoRa.packetRssi();
    lastSnr = LoRa.packetSnr();

    // Read packet header
    uint8_t destination = LoRa.read();
    uint8_t source = LoRa.read();
    uint8_t counter = LoRa.read();
    uint8_t type = LoRa.read();

    // Check if this packet is for us
    if (destination != GROUND_STATION_ID && destination != 0) {
        Serial.print("Packet not for us (dest: ");
        Serial.print(destination);
        return; // Not for us
    }

//    Serial.print("Source: ");
//    Serial.print(source);

    // Update connection status
    rocketConnectionActive = true;

    digitalWrite(LED_BLUE, HIGH); // Flash blue LED for received packet

    // Handle different response types
    if (type == 0x00 || type == static_cast<uint8_t>(ResponseCode::TELEMETRY_DATA)) {
        // Parse telemetry packet - Handle both the MessageType::TELEMETRY (0x00)
        // and ResponseCode::TELEMETRY_DATA (0x10)
//        Serial.println("Received telemetry data");

        if (packetSize >= 30) { // Min size for valid telemetry packet
            // Simple parsing
            lastTelemetry.timestamp = readUint32();
            lastTelemetry.rocketState = LoRa.read();
            lastTelemetry.altitude = readFloat();
            lastTelemetry.verticalSpeed = readFloat();
            lastTelemetry.acceleration = readFloat();
            lastTelemetry.temperature = readFloat();
            lastTelemetry.pressure = readFloat();
            lastTelemetry.batteryVoltage = readFloat();
            lastTelemetry.gpsSatellites = LoRa.read();
            lastTelemetry.gpsLatitude = readFloat();
            lastTelemetry.gpsLongitude = readFloat();
            lastTelemetry.gpsAltitude = readFloat();
            lastTelemetry.sensorStatus = LoRa.read();
            lastTelemetry.flags = LoRa.read();

            lastTelemetryTime = millis();

            // Print and log the telemetry
            printTelemetry();
            logTelemetry();
        }
    }

    // Read any remaining bytes
    while (LoRa.available()) {
        LoRa.read();
    }
}

void sendCommand(CommandCode code, uint8_t* payload, size_t length) {
    // Start LoRa packet
    LoRa.beginPacket();

    // Add header
    LoRa.write(ROCKET_ID);          // Destination = rocket
    LoRa.write(GROUND_STATION_ID);  // Source = ground station
    LoRa.write(packetCounter++);    // Packet counter
    LoRa.write(static_cast<uint8_t>(code)); // Command type

    // Add payload if provided
    if (payload != nullptr && length > 0) {
        LoRa.write(payload, length);
    }

    // Send packet
    LoRa.endPacket();

    // Store sequence number and time for ping calculation
    sequenceNumber = packetCounter - 1;
    if (code == CommandCode::PING) {
        lastPingTime = millis();
    }

    digitalWrite(LED_RED, HIGH); // Flash red LED for transmitted packet
    delay(50);
    digitalWrite(LED_RED, LOW);
}

void printTelemetry() {
    if (lastTelemetryTime == 0) {
        Serial.println("No telemetry data received yet");
        return;
    }

    // Create JSON with telemetry data
    String telemetryJson = "__tl__{";
    telemetryJson += "\"ts\":" + String(lastTelemetry.timestamp) + ",";
    telemetryJson += "\"age\":" + String((millis() - lastTelemetryTime) / 1000.0f, 1) + ",";
    telemetryJson += "\"state\":" + String(static_cast<uint8_t>(lastTelemetry.rocketState)) + ",";
    telemetryJson += "\"alt\":" + String(lastTelemetry.altitude, 1) + ",";
    telemetryJson += "\"vS\":" + String(lastTelemetry.verticalSpeed, 1) + ",";
    telemetryJson += "\"acc\":" + String(lastTelemetry.acceleration, 2) + ",";
    telemetryJson += "\"tem\":" + String(lastTelemetry.temperature, 1) + ",";
    telemetryJson += "\"pres\":" + String(lastTelemetry.pressure, 1) + ",";
    telemetryJson += "\"bV\":" + String(lastTelemetry.batteryVoltage, 2) + ",";
    telemetryJson += "\"gpsS\":" + String(lastTelemetry.gpsSatellites) + ",";
    telemetryJson += "\"gpsLat\":" + String(lastTelemetry.gpsLatitude, 6) + ",";
    telemetryJson += "\"gpsLong\":" + String(lastTelemetry.gpsLongitude, 6) + ",";
    telemetryJson += "\"gpsAlt\":" + String(lastTelemetry.gpsAltitude, 1) + ",";
    telemetryJson += "\"rssi\":" + String(lastRssi) + ",";
    telemetryJson += "\"snr\":" + String(lastSnr, 1) + ",";

    // Fix flags array formatting
    telemetryJson += "\"flgs\":[";
    bool hasFlag = false;
    if (lastTelemetry.flags & 0x01) { telemetryJson += "\"PARACHUTE\""; hasFlag = true; }
    if (lastTelemetry.flags & 0x02) { if(hasFlag) telemetryJson += ","; telemetryJson += "\"APOGEE\""; hasFlag = true; }
    if (lastTelemetry.flags & 0x04) { if(hasFlag) telemetryJson += ","; telemetryJson += "\"LAND\""; hasFlag = true; }
    if (lastTelemetry.flags & 0x08) { if(hasFlag) telemetryJson += ","; telemetryJson += "\"LOW_BAT\""; hasFlag = true; }
    if (lastTelemetry.flags & 0x10) { if(hasFlag) telemetryJson += ","; telemetryJson += "\"ERROR\""; hasFlag = true; }
    telemetryJson += "]}";

    Serial.println(telemetryJson);
}

void logTelemetry() {
    // Log to SD if available
    if (!logFile.isOpen()) {
        return;
    }

    // Write CSV record
    logFile.write(lastTelemetry.timestamp);
    logFile.write(",");
    logFile.write(lastTelemetry.rocketState);
    logFile.write(",");
    logFile.write(lastTelemetry.altitude);
    logFile.write(",");
    logFile.write(lastTelemetry.verticalSpeed);
    logFile.write(",");
    logFile.write(lastTelemetry.acceleration);
    logFile.write(",");
    logFile.write(lastTelemetry.temperature);
    logFile.write(",");
    logFile.write(lastTelemetry.pressure);
    logFile.write(",");
    logFile.write(lastTelemetry.batteryVoltage);
    logFile.write(",");
    logFile.write(lastTelemetry.gpsSatellites);
    logFile.write(",");
    logFile.write(lastTelemetry.gpsLatitude);
    logFile.write(",");
    logFile.write(lastTelemetry.gpsLongitude);
    logFile.write(",");
    logFile.write(lastRssi);
    logFile.write(",");
    logFile.write(lastSnr);

    // Flush every 10 records to avoid data loss
    if (++logCounter % 10 == 0) {
        logFile.flush();
    }
}

void updateLeds() {
    // Blue LED: connection status
    static unsigned long lastBlueToggle = 0;

    if (rocketConnectionActive) {
        // Fast blink for active connection
        if (millis() - lastBlueToggle > 500) {
            lastBlueToggle = millis();
            digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
        }
    } else {
        // Slow blink for no connection
        if (millis() - lastBlueToggle > 2000) {
            lastBlueToggle = millis();
            digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
        }
    }

    // Red LED: based on rocket state
    static unsigned long lastRedToggle = 0;

    if (lastTelemetryTime > 0) {
        switch (lastTelemetry.rocketState) {
            case static_cast<uint8_t>(RocketStatusCode::POWERED_FLIGHT):
            case static_cast<uint8_t>(RocketStatusCode::COASTING):
                // Solid on for flight
                digitalWrite(LED_RED, HIGH);
                break;

            case static_cast<uint8_t>(RocketStatusCode::ERROR):
                // SOS pattern for error
                static int sosPattern = 0;
                static unsigned long sosLastToggle = 0;

                if (millis() - sosLastToggle > 200) {
                    sosLastToggle = millis();

                    // SOS pattern: 3 short, 3 long, 3 short
                    if (sosPattern < 3 || sosPattern >= 9) {
                        // Short pulse (dot)
                        digitalWrite(LED_RED, sosPattern % 2 == 0);
                        sosPattern = (sosPattern + 1) % 12;
                    } else if (sosPattern >= 3 && sosPattern < 9) {
                        // Long pulse (dash)
                        if (sosPattern % 2 == 1) {
                            // Keep on for 3x as long for dash
                            if (millis() - sosLastToggle > 600) {
                                sosLastToggle = millis();
                                digitalWrite(LED_RED, LOW);
                                sosPattern++;
                            }
                        } else {
                            digitalWrite(LED_RED, HIGH);
                            sosPattern++;
                        }
                    }
                }
                break;

            default:
                // Default blink rate based on whether we're receiving telemetry
                if (millis() - lastTelemetryTime < 5000) {
                    // Slow blink for normal operation with recent telemetry
                    if (millis() - lastRedToggle > 1000) {
                        lastRedToggle = millis();
                        digitalWrite(LED_RED, !digitalRead(LED_RED));
                    }
                } else {
                    // Off when no telemetry
                    digitalWrite(LED_RED, LOW);
                }
                break;
        }
    } else {
        // No telemetry yet, keep LED off
        digitalWrite(LED_RED, LOW);
    }
}