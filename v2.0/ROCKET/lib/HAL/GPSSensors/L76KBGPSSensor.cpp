/**
 * L76KB-A58 GPS Sensor Implementation
 */

#include "L76KBGPSSensor.h"

L76KBGPSSensor::L76KBGPSSensor(HardwareSerial& serial, int8_t standbyPin, int8_t resetPin)
        : serial(serial), standbyPin(standbyPin), resetPin(resetPin) {

    // Initialize GPS data
    gpsData = {0};
    gpsData.valid = false;
}

L76KBGPSSensor::~L76KBGPSSensor() {
    // No need for special cleanup
}

SensorStatus L76KBGPSSensor::begin() {
    Serial.println("Initializing L76KB GPS sensor...");

    // Configure pins if provided
    if (standbyPin >= 0) {
        Serial.print("Configuring standby pin: ");
        Serial.println(standbyPin);
        pinMode(standbyPin, OUTPUT);
        digitalWrite(standbyPin, HIGH); // Active mode
    }

    if (resetPin >= 0) {
        Serial.print("Configuring reset pin: ");
        Serial.println(resetPin);
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, HIGH); // Normal operation
    }

    // Initialize serial communication
    Serial.println("Starting GPS serial communication at 9600 baud");
    serial.begin(9600);

    // Wait for the serial connection to be established
    delay(200);  // Increased delay

    // Reset module
    if (resetPin >= 0) {
        Serial.println("Resetting GPS module...");
        digitalWrite(resetPin, LOW);
        delay(200);
        digitalWrite(resetPin, HIGH);
        delay(500);
    }

    // Skip PMTK251 command to avoid baud rate change issues
    /*
    // Send basic configuration commands
    sendCommand("$PMTK251,38400*27\r\n"); // Set baud rate to 38400
    delay(100);

    // Reopen serial at the new baud rate
    serial.end();
    delay(100);
    serial.begin(38400);
    delay(100);
    */

    // Configure update rate
    Serial.println("Setting GPS update rate to 1Hz...");
    setUpdateRate(1); // 1Hz update rate

    // Enable all NMEA sentences we need
    Serial.println("Enabling NMEA sentences...");
    sendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

    // Initial update to check if sensor is responsive
    unsigned long startTime = millis();
    Serial.println("Waiting for initial GPS data...");
    bool receivedData = false;

    while (millis() - startTime < CONNECTION_TIMEOUT) {
        if (serial.available()) {
            char c = serial.read();
            Serial.print(c);  // Echo GPS data for debugging
            parseGPS(c);
            lastSerialActivity = millis();
            receivedData = true;
        }

        if (gps.charsProcessed() > 100) {
            // We got some data, so likely the GPS is responding
            Serial.println("\nGPS is responding");
            status = SensorStatus::OK;
            return status;
        }

        // Add a short delay to prevent busy-waiting
        delay(5);
    }

    if (!receivedData) {
        Serial.println("No data received from GPS");
    } else {
        Serial.println("Received some data but not enough");
    }

    // If we got here, the GPS didn't respond with enough data within the timeout
    status = SensorStatus::INITIALIZATION_ERROR;
    Serial.println("GPS initialization failed - timeout waiting for data");
    return status;
}

SensorStatus L76KBGPSSensor::update() {
    unsigned long startTime = millis();
    bool newData = false;

    // Process all available data
    while (serial.available() && millis() - startTime < 100) { // Read for up to 100ms
        char c = serial.read();
        parseGPS(c);
        newData = true;
        lastSerialActivity = millis();
    }

    // Update GPS data if we have a valid position
    if (gps.location.isValid()) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.valid = true;
        gpsData.age = gps.location.age();

        if (gps.altitude.isValid()) {
            gpsData.altitude = gps.altitude.meters();
        }

        if (gps.speed.isValid()) {
            gpsData.speed = gps.speed.mps();
        }

        if (gps.course.isValid()) {
            gpsData.course = gps.course.deg();
        }

        if (gps.satellites.isValid()) {
            gpsData.satellites = gps.satellites.value();
        }

        if (gps.hdop.isValid()) {
            gpsData.hdop = gps.hdop.hdop();
        }

        if (gps.date.isValid() && gps.time.isValid()) {
            gpsData.year = gps.date.year();
            gpsData.month = gps.date.month();
            gpsData.day = gps.date.day();
            gpsData.hour = gps.time.hour();
            gpsData.minute = gps.time.minute();
            gpsData.second = gps.time.second();
        }
    } else if (newData) {
        // We got data but no valid position
        gpsData.valid = false;
    }

    // Check if we've lost communication with the GPS
    if (millis() - lastSerialActivity > 5000) {
        status = SensorStatus::COMMUNICATION_ERROR;
    } else {
        status = SensorStatus::OK;
        lastReadingTime = millis();
    }

    return status;
}

bool L76KBGPSSensor::isOperational() {
    return status == SensorStatus::OK && (millis() - lastSerialActivity < 5000);
}

SensorStatus L76KBGPSSensor::getStatus() const {
    return status;
}

const char* L76KBGPSSensor::getName() const {
    return "L76KB-A58";
}

unsigned long L76KBGPSSensor::getLastReadingTime() const {
    return lastReadingTime;
}

GPSData L76KBGPSSensor::getGPSData() {
    return gpsData;
}

bool L76KBGPSSensor::hasPositionFix() {
    return gpsData.valid && gpsData.age < 2000; // Valid fix less than 2 seconds old
}

unsigned long L76KBGPSSensor::getFixAge() {
    return gpsData.valid ? gpsData.age : ULONG_MAX;
}

uint8_t L76KBGPSSensor::getSatelliteCount() {
    return gpsData.satellites;
}

bool L76KBGPSSensor::enableLowPowerMode() {
    if (standbyPin < 0) {
        return false; // Not supported without standby pin
    }

    digitalWrite(standbyPin, LOW); // Put into standby mode
    lowPowerMode = true;
    return true;
}

bool L76KBGPSSensor::disableLowPowerMode() {
    if (standbyPin < 0 || !lowPowerMode) {
        return false; // Not in low power mode or not supported
    }

    digitalWrite(standbyPin, HIGH); // Wake up from standby mode
    lowPowerMode = false;

    // Give some time for the GPS to wake up
    delay(100);

    return true;
}

bool L76KBGPSSensor::reset() {
    if (resetPin < 0) {
        return false; // Not supported without reset pin
    }

    // Reset sequence
    digitalWrite(resetPin, LOW);
    delay(200);
    digitalWrite(resetPin, HIGH);
    delay(500);

    // Clear any existing data
    while (serial.available()) {
        serial.read();
    }

    // Reconfigure the module
    setUpdateRate(1); // 1Hz update rate
    sendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

    return true;
}

bool L76KBGPSSensor::sendCommand(const char* cmd) {
    serial.print(cmd);
    return true; // We can't easily verify if command was accepted
}

bool L76KBGPSSensor::setUpdateRate(int rateHz) {
    switch (rateHz) {
        case 1:
            return sendCommand("$PMTK220,1000*1F\r\n");
        case 2:
            return sendCommand("$PMTK220,500*2B\r\n");
        case 5:
            return sendCommand("$PMTK220,200*2C\r\n");
        case 10:
            return sendCommand("$PMTK220,100*2F\r\n");
        default:
            return false; // Unsupported rate
    }
}

void L76KBGPSSensor::parseGPS(char c) {
    gps.encode(c);
}

bool L76KBGPSSensor::waitForResponse(const char* expectedResponse, unsigned long timeout) {
    unsigned long startTime = millis();
    String response = "";

    while (millis() - startTime < timeout) {
        if (serial.available()) {
            char c = serial.read();
            response += c;

            if (response.endsWith("\r\n")) {
                if (response.indexOf(expectedResponse) >= 0) {
                    return true;
                }
                response = ""; // Reset for next line
            }
        }
    }

    return false; // Timeout
}