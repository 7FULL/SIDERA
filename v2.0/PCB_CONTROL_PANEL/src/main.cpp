/**
 * Ground Station Control Panel for Rocket System
 *
 * This code runs on a RP2040-based board with the same hardware as the rocket,
 * providing a ground control interface to monitor telemetry and send commands.
 */

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
#define LORA_SPREADING_FACTOR 7
#define LORA_BANDWIDTH 125E3
#define LORA_CODING_RATE 5
#define LORA_TX_POWER 17

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
void checkConnection();
void printHelp();
bool parseFloat(float& value);
bool parseInt(int& value);

void setup() {
    // Initialize serial for debugging and user interface
    Serial.begin(115200);
    while (!Serial && millis() < 5000);

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

    // Initialize SPI
    SPI1.setMISO(SPI1_MISO);
    SPI1.setMOSI(SPI1_MOSI);
    SPI1.setSCK(SPI1_SCK);
    SPI1.begin();

    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    // Initialize LoRa and SD
    initLoRa();
    initSD();

    // Print instructions
    printHelp();
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
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

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

    // Create a new log file with timestamp
    sprintf(logFileName, "LOG_%lu.CSV", millis());
    if (!logFile.open(logFileName, O_WRITE | O_CREAT)) {
        Serial.println("Error creating log file");
        return;
    }

    // Write header
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
            case '?':
            case 'h':
                printHelp();
                break;

            case 'p':
                // Ping command
                Serial.println("Sending PING command...");
                sendCommand(CommandCode::PING);
                break;

            case 's':
                // Get status command
                Serial.println("Requesting rocket status...");
                sendCommand(CommandCode::GET_STATUS);
                break;

            case 'a':
                // Arm rocket
                Serial.println("Arming rocket...");
//                sendCommand(CommandCode::ARM_ROCKET);
                break;

            case 'd':
                // Disarm rocket
                Serial.println("Disarming rocket...");
//                sendCommand(CommandCode::DISARM_ROCKET);
                break;

            case 'l':
                // Launch command
                Serial.println("WARNING: SENDING LAUNCH COMMAND!");
                Serial.println("Are you sure? (y/n)");
                sendCommand(CommandCode::WAKE_UP_COMMAND);
                break;

            case 'x':
                // Abort command
                Serial.println("Sending ABORT command...");
                sendCommand(CommandCode::ABORT_COMMAND);
                break;

            case 'c':
                // Calibrate sensors
                Serial.println("Calibrating sensors...");
                sendCommand(CommandCode::CALIBRATE_SENSORS);
                break;

            case 'r':
                // Run diagnostics
                Serial.println("Running diagnostics...");
                sendCommand(CommandCode::RUN_DIAGNOSTICS);
                break;

            case 'e':
                // Deploy parachute (EMERGENCY)
                Serial.println("WARNING: EMERGENCY PARACHUTE DEPLOYMENT!");
                Serial.println("Are you sure? (y/n)");
                while (!Serial.available());
                if (Serial.read() == 'y') {
                    Serial.println("DEPLOYING PARACHUTE");
//                    sendCommand(CommandCode::FORCE_DEPLOY_PARACHUTE);
                } else {
                    Serial.println("Deployment aborted");
                }
                break;

            case 'i':
                // Print last telemetry data
                printTelemetry();
                break;

            case 'w':
                // Set parameter
            {
                Serial.println("Enter parameter ID (0-255):");
                int parameterId;
                if (!parseInt(parameterId) || parameterId < 0 || parameterId > 255) {
                    Serial.println("Invalid parameter ID");
                    break;
                }

                Serial.println("Enter parameter value (integer):");
                int parameterValue;
                if (!parseInt(parameterValue)) {
                    Serial.println("Invalid parameter value");
                    break;
                }

                uint8_t payload[3];
                payload[0] = (uint8_t)parameterId;
                payload[1] = (parameterValue >> 8) & 0xFF;
                payload[2] = parameterValue & 0xFF;

                Serial.println("Setting parameter...");
//                sendCommand(CommandCode::SET_PARAMETER, payload, 3);
            }
                break;

            case 'g':
                // Get parameter
            {
                Serial.println("Enter parameter ID (0-255):");
                int parameterId;
                if (!parseInt(parameterId) || parameterId < 0 || parameterId > 255) {
                    Serial.println("Invalid parameter ID");
                    break;
                }

                uint8_t payload[1];
                payload[0] = (uint8_t)parameterId;

                Serial.println("Getting parameter...");
//                sendCommand(CommandCode::GET_PARAMETER, payload, 1);
            }
                break;

            case 'z':
                // Reset system (dangerous)
                Serial.println("WARNING: RESETTING ROCKET SYSTEM!");
                Serial.println("Are you sure? (y/n)");
                while (!Serial.available());
                if (Serial.read() == 'y') {
                    Serial.println("RESETTING ROCKET SYSTEM");
//                    sendCommand(CommandCode::RESET_SYSTEM);
                } else {
                    Serial.println("Reset aborted");
                }
                break;

            case '\r':
            case '\n':
                // Ignore newlines
                break;

            default:
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                printHelp();
                break;
        }

        // Clear any remaining characters
        while (Serial.available()) {
            Serial.read();
        }
    }
}

void checkConnection() {
    // Check if we haven't received any message for a while
    unsigned long timeWithoutMessage = millis() - lastTelemetryTime;

    if (rocketConnectionActive && timeWithoutMessage > 10000) {
        // If no message for 10 seconds, consider connection lost
        rocketConnectionActive = false;
        Serial.println("\n!!! CONNECTION LOST !!!");
        Serial.println("No telemetry for 10 seconds");
    }

    // Send a ping periodically if we haven't received telemetry in a while
    static unsigned long lastPingSent = 0;
    if (timeWithoutMessage > 5000 && (millis() - lastPingSent) > 5000) {
        lastPingSent = millis();
        Serial.println("Sending automatic ping to check connection...");
        sendCommand(CommandCode::PING);
    }
}

// Helper function to print rocket state in human-readable form
void printRocketState(uint8_t state) {
    switch (static_cast<RocketStatusCode>(state)) {
        case RocketStatusCode::INIT:
            Serial.println("INITIALIZING");
            break;
        case RocketStatusCode::GROUND_IDLE:
            Serial.println("IDLE");
            break;
        case RocketStatusCode::READY:
            Serial.println("READY");
            break;
        case RocketStatusCode::POWERED_FLIGHT:
            Serial.println("POWERED FLIGHT");
            break;
        case RocketStatusCode::COASTING:
            Serial.println("COASTING");
            break;
        case RocketStatusCode::APOGEE:
            Serial.println("APOGEE");
            break;
        case RocketStatusCode::DESCENT:
            Serial.println("DESCENT");
            break;
        case RocketStatusCode::PARACHUTE_DESCENT:
            Serial.println("PARACHUTE DEPLOYED");
            break;
        case RocketStatusCode::LANDED:
            Serial.println("LANDED");
            break;
        case RocketStatusCode::ERROR:
            Serial.println("ERROR");
            break;
        default:
            Serial.print("UNKNOWN (");
            Serial.print(state);
            Serial.println(")");
            break;
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

// Helper function to print formatted time (days:hours:minutes:seconds)
void printFormattedTime(uint32_t seconds) {
    uint32_t days = seconds / 86400;
    seconds %= 86400;
    uint32_t hours = seconds / 3600;
    seconds %= 3600;
    uint32_t minutes = seconds / 60;
    seconds %= 60;

    if (days > 0) {
        Serial.print(days);
        Serial.print("d ");
    }

    if (hours > 0 || days > 0) {
        Serial.print(hours);
        Serial.print("h ");
    }

    Serial.print(minutes);
    Serial.print("m ");
    Serial.print(seconds);
    Serial.println("s");
}

void printHelp() {
    Serial.println("\n==== Ground Station Commands ====");
    Serial.println("p - Ping rocket");
    Serial.println("s - Get rocket status");
    Serial.println("i - Print last telemetry data");
    Serial.println("a - Arm rocket");
    Serial.println("d - Disarm rocket");
    Serial.println("l - Launch (start countdown)");
    Serial.println("x - Abort countdown/mission");
    Serial.println("e - Emergency parachute deployment");
    Serial.println("c - Calibrate sensors");
    Serial.println("r - Run diagnostics");
    Serial.println("w - Set parameter");
    Serial.println("g - Get parameter");
    Serial.println("z - Reset rocket system");
    Serial.println("h/? - Show this help");
    Serial.println("================================");
}

// Helper to parse float from serial
bool parseFloat(float& value) {
    String input = "";
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (input.length() > 0) {
                    value = input.toFloat();
                    return true;
                }
                return false;
            } else if (isdigit(c) || c == '.' || c == '-') {
                input += c;
                Serial.print(c); // Echo back
            }
        }
    }
}

// Helper to parse integer from serial
bool parseInt(int& value) {
    String input = "";
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (input.length() > 0) {
                    value = input.toInt();
                    return true;
                }
                return false;
            } else if (isdigit(c) || c == '-') {
                input += c;
                Serial.print(c); // Echo back
            }
        }
    }
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
        Serial.println("Received telemetry data");

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
    else switch (static_cast<ResponseCode>(type)) {
            case ResponseCode::ACK:
                Serial.println("Received ACK");
                if (packetSize > 4) {
                    // ACK with timestamp (ping response)
                    uint32_t timestamp = 0;
                    for (int i = 0; i < 4 && LoRa.available(); i++) {
                        timestamp = (timestamp << 8) | LoRa.read();
                    }
                    // Calculate round-trip time
                    unsigned long rtt = millis() - lastPingTime;
                    Serial.print("Ping RTT: ");
                    Serial.print(rtt);
                    Serial.println(" ms");
                    lastPingResponseTime = millis();
                }
                break;

            case ResponseCode::NACK:
                Serial.println("Received NACK - Command failed");
                break;

            case ResponseCode::STATUS_DATA:
                if (packetSize >= 14) {
                    rocketStatus = LoRa.read(); // Status code

                    // Parse battery info
                    uint16_t batteryVoltage = (LoRa.read() << 8) | LoRa.read();
                    uint8_t batteryPercentage = LoRa.read();

                    // Parse uptime
                    uint32_t uptime = 0;
                    for (int i = 0; i < 4; i++) {
                        uptime = (uptime << 8) | LoRa.read();
                    }

                    // Parse storage
                    uint32_t freeSpace = 0;
                    for (int i = 0; i < 4; i++) {
                        freeSpace = (freeSpace << 8) | LoRa.read();
                    }

                    // Parse GPS fix and other flags
                    bool gpsFix = LoRa.read() > 0;
                    uint8_t sensorFlags = LoRa.read();

                    // Display the status
                    Serial.println("\n----- Rocket Status -----");
                    Serial.print("State: ");
                    printRocketState(rocketStatus);
                    Serial.print("Battery: ");
                    Serial.print(batteryVoltage / 100.0f, 2);
                    Serial.print("V (");
                    Serial.print(batteryPercentage);
                    Serial.println("%)");
                    Serial.print("Uptime: ");
                    printFormattedTime(uptime);
                    Serial.print("Free Space: ");
                    Serial.print(freeSpace / 1024);
                    Serial.println("KB");
                    Serial.print("GPS Fix: ");
                    Serial.println(gpsFix ? "YES" : "NO");
                    Serial.print("Sensor Status: 0x");
                    Serial.println(sensorFlags, HEX);
                    Serial.print("Signal: RSSI ");
                    Serial.print(lastRssi);
                    Serial.print("dBm, SNR ");
                    Serial.print(lastSnr, 1);
                    Serial.println("dB");
                    Serial.println("------------------------");
                }
                break;

            case ResponseCode::DIAGNOSTIC_RESULT:
                if (packetSize > 5) {
                    uint8_t testCount = LoRa.read();
                    Serial.println("\n----- Diagnostic Results -----");
                    Serial.print("Tests run: ");
                    Serial.println(testCount);

                    for (int i = 0; i < testCount && LoRa.available(); i++) {
                        bool passed = LoRa.read() > 0;
                        uint8_t nameLength = LoRa.read();

                        String testName = "";
                        for (int j = 0; j < nameLength && LoRa.available(); j++) {
                            testName += (char)LoRa.read();
                        }

                        Serial.print(i+1);
                        Serial.print(". ");
                        Serial.print(testName);
                        Serial.print(": ");
                        Serial.println(passed ? "PASS" : "FAIL");

                        if (!passed) {
                            // Read error message
                            uint8_t errorLength = LoRa.read();
                            String errorMsg = "";
                            for (int j = 0; j < errorLength && LoRa.available(); j++) {
                                errorMsg += (char)LoRa.read();
                            }
                            Serial.print("   Error: ");
                            Serial.println(errorMsg);
                        }
                    }
                    Serial.println("-----------------------------");
                }
                break;

            case ResponseCode::EVENT_NOTIFICATION:
                if (packetSize > 5) {
                    uint8_t eventCode = LoRa.read();

                    String eventDescription = "";
                    while (LoRa.available()) {
                        eventDescription += (char)LoRa.read();
                    }

                    Serial.println("\n!!! EVENT NOTIFICATION !!!");
                    Serial.print("Event Code: 0x");
                    Serial.println(eventCode, HEX);
                    Serial.print("Description: ");
                    Serial.println(eventDescription);
                    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!");
                }
                break;

            case ResponseCode::ERROR_MESSAGE:
            {
                String errorMsg = "";
                while (LoRa.available()) {
                    errorMsg += (char)LoRa.read();
                }

                Serial.println("\n!!! ERROR MESSAGE !!!");
                Serial.println(errorMsg);
                Serial.println("!!!!!!!!!!!!!!!!!!!!");
            }
                break;

            case ResponseCode::PARAMETER_VALUE:
                if (packetSize > 7) {
                    uint8_t paramId = LoRa.read();
                    uint16_t paramValue = (LoRa.read() << 8) | LoRa.read();

                    Serial.println("\n----- Parameter Value -----");
                    Serial.print("Parameter ID: ");
                    Serial.println(paramId);
                    Serial.print("Value: ");
                    Serial.println(paramValue);
                    Serial.println("--------------------------");
                }
                break;

            default:
                Serial.print("Unknown packet type: 0x");
                Serial.println(type, HEX);
                break;
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
    // Only print if we have received telemetry data
    if (lastTelemetryTime == 0) {
        Serial.println("No telemetry data received yet");
        return;
    }

    Serial.println("\n----- Telemetry Data -----");
    Serial.print("Time: ");
    Serial.print(lastTelemetry.timestamp);
    Serial.print(" (Age: ");
    Serial.print((millis() - lastTelemetryTime) / 1000.0f, 1);
    Serial.println("s)");

    Serial.print("State: ");
    printRocketState(lastTelemetry.rocketState);

    Serial.print("Altitude: ");
    Serial.print(lastTelemetry.altitude, 1);
    Serial.println(" m");

    Serial.print("Vertical Speed: ");
    Serial.print(lastTelemetry.verticalSpeed, 1);
    Serial.println(" m/s");

    Serial.print("Acceleration: ");
    Serial.print(lastTelemetry.acceleration, 2);
    Serial.println(" m/s²");

    Serial.print("Temperature: ");
    Serial.print(lastTelemetry.temperature, 1);
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(lastTelemetry.pressure, 1);
    Serial.println(" hPa");

    Serial.print("Battery: ");
    Serial.print(lastTelemetry.batteryVoltage, 2);
    Serial.println(" V");

    Serial.print("GPS: ");
    if (lastTelemetry.gpsSatellites > 0) {
        Serial.print(lastTelemetry.gpsLatitude, 6);
        Serial.print(", ");
        Serial.print(lastTelemetry.gpsLongitude, 6);
        Serial.print(" (");
        Serial.print(lastTelemetry.gpsSatellites);
        Serial.println(" sats)");
    } else {
        Serial.println("No fix");
    }

    Serial.print("Signal: RSSI ");
    Serial.print(lastRssi);
    Serial.print("dBm, SNR ");
    Serial.print(lastSnr, 1);
    Serial.println("dB");

    // Print any flags
    if (lastTelemetry.flags > 0) {
        Serial.print("Flags: ");
        if (lastTelemetry.flags & 0x01) Serial.print("PARACHUTE_DEPLOYED ");
        if (lastTelemetry.flags & 0x02) Serial.print("APOGEE_DETECTED ");
        if (lastTelemetry.flags & 0x04) Serial.print("LANDED ");
        if (lastTelemetry.flags & 0x08) Serial.print("LOW_BATTERY ");
        if (lastTelemetry.flags & 0x10) Serial.print("ERROR_CONDITION ");
        Serial.println();
    }

    Serial.println("-------------------------");
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