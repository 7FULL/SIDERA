/**
 * ATGM336H GPS Sensor Implementation
 */

#include "ATGM336HGPSSensor.h"

ATGM336HGPSSensor::ATGM336HGPSSensor(HardwareSerial& serial, int8_t standbyPin, int8_t resetPin)
        : serial(serial), standbyPin(standbyPin), resetPin(resetPin) {

    // Initialize GPS data
    gpsData = {0};
    gpsData.valid = false;
}

ATGM336HGPSSensor::~ATGM336HGPSSensor() {
    // No need for special cleanup
}

SensorStatus ATGM336HGPSSensor::begin() {
    // Configure pins if provided
    if (standbyPin >= 0) {
        pinMode(standbyPin, OUTPUT);
        digitalWrite(standbyPin, HIGH); // Active mode
    }

    if (resetPin >= 0) {
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, HIGH); // Normal operation
    }

    // Initialize serial communication (9600 baud is typical for GPS)
    serial.begin(9600);

    // Wait for the serial connection to be established
    delay(100);

    // Reset module if needed
    if (resetPin >= 0) {
        digitalWrite(resetPin, LOW);
        delay(200);
        digitalWrite(resetPin, HIGH);
        delay(500);
    }

    // Send basic configuration commands
    // The ATGM336H typically uses similar PMTK commands to the L76KB

    // Set update rate to 1Hz
    setUpdateRate(1);

    // Enable all NMEA sentences we need
    sendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

    // Initial update to check if sensor is responsive
    unsigned long startTime = millis();
    while (millis() - startTime < CONNECTION_TIMEOUT) {
        if (serial.available()) {
            parseGPS(serial.read());
            lastSerialActivity = millis();
        }

        if (gps.charsProcessed() > 100) {
            // We got some data, so likely the GPS is responding
            status = SensorStatus::OK;
            return status;
        }
    }

    // If we got here, the GPS didn't respond within the timeout
    status = SensorStatus::INITIALIZATION_ERROR;
    return status;
}

SensorStatus ATGM336HGPSSensor::update() {
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

bool ATGM336HGPSSensor::isOperational() {
    return status == SensorStatus::OK && (millis() - lastSerialActivity < 5000);
}

SensorStatus ATGM336HGPSSensor::getStatus() const {
    return status;
}

const char* ATGM336HGPSSensor::getName() const {
    return "ATGM336H";
}

unsigned long ATGM336HGPSSensor::getLastReadingTime() const {
    return lastReadingTime;
}

GPSData ATGM336HGPSSensor::getGPSData() {
    return gpsData;
}

bool ATGM336HGPSSensor::hasPositionFix() {
    return gpsData.valid && gpsData.age < 2000; // Valid fix less than 2 seconds old
}

unsigned long ATGM336HGPSSensor::getFixAge() {
    return gpsData.valid ? gpsData.age : ULONG_MAX;
}

uint8_t ATGM336HGPSSensor::getSatelliteCount() {
    return gpsData.satellites;
}

bool ATGM336HGPSSensor::enableLowPowerMode() {
    if (standbyPin < 0) {
        return false; // Not supported without standby pin
    }

    digitalWrite(standbyPin, LOW); // Put into standby mode
    lowPowerMode = true;
    return true;
}

bool ATGM336HGPSSensor::disableLowPowerMode() {
    if (standbyPin < 0 || !lowPowerMode) {
        return false; // Not in low power mode or not supported
    }

    digitalWrite(standbyPin, HIGH); // Wake up from standby mode
    lowPowerMode = false;

    // Give some time for the GPS to wake up
    delay(100);

    return true;
}

bool ATGM336HGPSSensor::reset() {
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

bool ATGM336HGPSSensor::sendCommand(const char* cmd) {
    serial.print(cmd);
    return true; // We can't easily verify if command was accepted
}

bool ATGM336HGPSSensor::setUpdateRate(int rateHz) {
    switch (rateHz) {
        case 1:
            return sendCommand("$PMTK220,1000*1F\r\n");
        case 5:
            return sendCommand("$PMTK220,200*2C\r\n");
        case 10:
            return sendCommand("$PMTK220,100*2F\r\n");
        default:
            return false; // Unsupported rate
    }
}

void ATGM336HGPSSensor::parseGPS(char c) {
    gps.encode(c);
}

bool ATGM336HGPSSensor::waitForResponse(const char* expectedResponse, unsigned long timeout) {
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