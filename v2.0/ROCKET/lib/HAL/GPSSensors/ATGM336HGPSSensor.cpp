/**
 * ATGM336H GPS Sensor Implementation
 */

#include "ATGM336HGPSSensor.h"
#include "Config.h"

ATGM336HGPSSensor::ATGM336HGPSSensor(HardwareSerial& serial, int8_t standbyPin, int8_t resetPin)
        : serial(serial),
          standbyPin(standbyPin),
          resetPin(resetPin),
          lowPowerMode(false),
          lastDataTime(0) {

    // Initialize GPS data
    gpsData = {0};
    gpsData.valid = false;
}

ATGM336HGPSSensor::~ATGM336HGPSSensor() {
    // No special cleanup needed
}

bool ATGM336HGPSSensor::sendCommand(const char* cmd) {
    serial.print(cmd);
    Serial.print("ATGM336HGP: Sent command: ");
    Serial.print(cmd);
    return true; // We can't easily verify if command was accepted
}

bool ATGM336HGPSSensor::setUpdateRate(int rateHz) {
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

SensorStatus ATGM336HGPSSensor::begin() {
    Serial.println("ATGM336HGP: Starting initialization...");

    // Configure standby pin if provided
    if (standbyPin >= 0) {
        Serial.print("ATGM336HGP: Configuring standby pin ");
        Serial.println(standbyPin);
        pinMode(standbyPin, OUTPUT);
        digitalWrite(standbyPin, HIGH); // Set to active mode
    }

    // Configure reset pin if provided
    if (resetPin >= 0) {
        Serial.print("ATGM336HGP: Configuring reset pin ");
        Serial.println(resetPin);
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, HIGH); // Set to normal operation
    }

    // FIXED: Start GPS serial communication at correct baud rate
    Serial.println("ATGM336HGP: Starting serial communication at 9600 baud");
    serial.begin(9600);  // Changed from 115200 to 9600

    // Small delay to ensure serial is ready
    delay(100);

    // Reset module if reset pin is available
    if (resetPin >= 0) {
        Serial.println("ATGM336HGP: Performing reset sequence");
        digitalWrite(resetPin, LOW);
        delay(100);
        digitalWrite(resetPin, HIGH);
        delay(500);
    }

    // Configure update rate
    Serial.println("ATGM336HGP: Setting update rate to 1Hz");
    sendCommand("$PMTK220,1000*1F\r\n");

    // Enable all needed NMEA sentences
    Serial.println("ATGM336HGP: Enabling NMEA sentences");
    sendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

    Serial.println("ATGM336HGP: Waiting for GPS fix...");

    // FIXED: Reduced timeout from 5 minutes to 60 seconds for faster startup
    unsigned long startTime = millis();
    bool receivedData = false;
    unsigned long msToWait = GPS_INIT_TIMEOUT; // Changed from 300000 (5 min) to 60000 (1 min)

    while ((millis() - startTime < msToWait) && !receivedData) {
        // Show progress every 10 seconds
        if ((millis() - startTime) % GPS_PROGRESS_INTERVAL == 0) {
            Serial.print("ATGM336HGP: Waiting for fix... ");
            Serial.print((millis() - startTime) / 1000);
            Serial.println(" seconds elapsed");
        }

        while (serial.available() > 0) {
            char c = serial.read();
            gps.encode(c);
            lastDataTime = millis();

            // Check if we have a valid location and a valid fix
            if (gps.location.isValid() &&
                gps.location.age() < 2000 &&
                gps.hdop.isValid() &&
                gps.hdop.hdop() < 20 &&
                gps.satellites.isValid() &&
                gps.satellites.value() >= 4 &&
                gps.altitude.isValid() &&
                gps.speed.isValid() &&
                gps.course.isValid() &&
                gps.date.isValid() &&
                gps.time.isValid() &&
                hasPositionFix()) {

                receivedData = true;
                displayDebugInfo();
                break;
            }
        }

        // Small delay to prevent excessive CPU usage
        delay(100);
    }

    // IMPROVED: Better error handling for GPS initialization
    if (!receivedData) {
        Serial.println("ATGM336HGP: No GPS fix obtained within timeout");
        Serial.println("ATGM336HGP: Continuing without GPS fix - GPS will work in background");
        // Don't fail initialization - GPS can work in background
    } else {
        Serial.println("ATGM336HGP: GPS fix obtained successfully");
    }

    // Mark as initialized regardless of GPS fix status
    gpsInitialized = true;
    lastReadingTime = millis();
    status = SensorStatus::OK;

    Serial.println("ATGM336HGP: Initialization complete");
    return status;
}

SensorStatus ATGM336HGPSSensor::update() {
    // Read data from GPS module
    while (serial.available() > 0) {
        char c = serial.read();
        gps.encode(c);
        lastDataTime = millis();
    }

    // Update GPS data if we have a valid position
    if (gps.location.isValid()) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.valid = true;
        gpsData.age = gps.location.age();

        // Update all other data
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
    } else {
        // We got data but no valid position
        gpsData.valid = false;
    }

    // Check if we've lost communication with the GPS
    if (millis() - lastDataTime > 5000) {
        status = SensorStatus::COMMUNICATION_ERROR;
    } else {
        status = SensorStatus::OK;
        lastReadingTime = millis();
    }

    // Display debug info periodically (every 15 seconds)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 15000) {
        lastDebugTime = millis();

        #ifdef ENABLE_ATGM336H_DEBUG
        displayDebugInfo();
        #endif
    }

    return status;
}

bool ATGM336HGPSSensor::isOperational() {
    return (status == SensorStatus::OK) && (millis() - lastDataTime < 5000);
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
    return gps.location.isValid();
}

unsigned long ATGM336HGPSSensor::getFixAge() {
    return gps.location.isValid() ? gps.location.age() : ULONG_MAX;
}

uint8_t ATGM336HGPSSensor::getSatelliteCount() {
    return gps.satellites.isValid() ? gps.satellites.value() : 0;
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

    return true;
}

String ATGM336HGPSSensor::getHDOPDescription(float hdopValue) const {
    if (hdopValue < 1.0) {
        return "Ideal (Highest possible confidence level)";
    } else if (hdopValue < 2.0) {
        return "Excellent (Position measurements are considered accurate)";
    } else if (hdopValue < 5.0) {
        return "Good (Position measurements suitable for navigation)";
    } else if (hdopValue < 10.0) {
        return "Moderate (Position measurements can be used for calculations)";
    } else if (hdopValue < 20.0) {
        return "Fair (Low confidence level, measurements should be discarded)";
    } else {
        return "Poor (Very low confidence level, measurements unreliable)";
    }
}

void ATGM336HGPSSensor::displayDebugInfo() {
    Serial.println("\n----- ATGM336H GPS Status -----");

    Serial.print("Fix: ");
    Serial.println(hasPositionFix() ? "Yes" : "No");

    Serial.print("Satellites: ");
    if (gps.satellites.isValid()) {
        Serial.println(gps.satellites.value());
    } else {
        Serial.println("Not Available");
    }

    Serial.print("Location: ");
    if (gps.location.isValid()) {
        Serial.print("Lat: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", Lng: ");
        Serial.println(gps.location.lng(), 6);
    } else {
        Serial.println("Not Available");
    }

    Serial.print("Altitude: ");
    if (gps.altitude.isValid()) {
        Serial.print(gps.altitude.meters());
        Serial.println(" m");
    } else {
        Serial.println("Not Available");
    }

    Serial.print("Speed: ");
    if (gps.speed.isValid()) {
        Serial.print(gps.speed.kmph());
        Serial.println(" km/h");
    } else {
        Serial.println("Not Available");
    }

    if (gps.hdop.isValid()) {
        Serial.print("HDOP: ");
        Serial.println(gps.hdop.hdop(), 1);
        Serial.print("Meaning: ");
        Serial.println(getHDOPDescription(gps.hdop.hdop()));
    } else {
        Serial.println("HDOP: Not Available");
    }

    Serial.print("Characters processed: ");
    Serial.println(gps.charsProcessed());

    Serial.print("Sentences with fix: ");
    Serial.println(gps.sentencesWithFix());

    Serial.print("Failed checksum: ");
    Serial.println(gps.failedChecksum());

    Serial.println("----------------------------");
}