/**
 * L76KB-A58 GPS Sensor Implementation
 */

#include "L76KBGPSSensor.h"

L76KBGPSSensor::L76KBGPSSensor(HardwareSerial& serial, int8_t standbyPin, int8_t resetPin)
        : serial(serial),
          standbyPin(standbyPin),
          resetPin(resetPin),
          lowPowerMode(false),
          gpsInitialized(false),
          lastDataTime(0) {

    // Initialize GPS data
    gpsData = {0};
    gpsData.valid = false;
}

L76KBGPSSensor::~L76KBGPSSensor() {
    // No special cleanup needed
}

SensorStatus L76KBGPSSensor::begin() {
    Serial.println("L76KB: Starting initialization...");

    // Configure standby pin if provided
    if (standbyPin >= 0) {
        Serial.print("L76KB: Configuring standby pin ");
        Serial.println(standbyPin);
        pinMode(standbyPin, OUTPUT);
        digitalWrite(standbyPin, HIGH); // Set to active mode
    }

    // Configure reset pin if provided
    if (resetPin >= 0) {
        Serial.print("L76KB: Configuring reset pin ");
        Serial.println(resetPin);
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, HIGH); // Set to normal operation
    }

    // Start GPS serial communication
    Serial.println("L76KB: Starting serial communication at 9600 baud");
    serial.begin(9600);

    // Small delay to ensure serial is ready
    delay(100);

    // Reset module if reset pin is available
    if (resetPin >= 0) {
        Serial.println("L76KB: Performing reset sequence");
        digitalWrite(resetPin, LOW);
        delay(100);
        digitalWrite(resetPin, HIGH);
        delay(500);
    }

    // Configure update rate
    Serial.println("L76KB: Setting update rate to 1Hz");
    sendCommand("$PMTK220,1000*1F\r\n");

    // Enable all needed NMEA sentences
    Serial.println("L76KB: Enabling NMEA sentences");
    sendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

    Serial.println("L76KB: Waiting for GPS fix...");

    // Wait until we receive a valid GPS fix or timeout after 2 minutes
    unsigned long startTime = millis();
    bool receivedData = false;
    unsigned long msToWait = 300000; // 5 minutos

    while ((millis() - startTime < msToWait) && !receivedData) {
//        Serial.print("Seconds elapsed: ");
//        Serial.print((millis() - startTime) / 1000);
//        Serial.print(" / ");
//        Serial.print(msToWait / 1000);
//        Serial.println(" seconds");

        while (serial.available() > 0) {
            char c = serial.read();
            gps.encode(c);
            lastDataTime = millis();

//             Serial.print(c);

            // Check if we have a valid location and a valid fix and a valid HDOP
            if (
                    gps.location.isValid()
                    && gps.location.age() < 2000
                    && gps.hdop.isValid()
                    && gps.hdop.hdop() < 20
                    && gps.satellites.isValid()
                    && gps.satellites.value() >= 4
                    && gps.altitude.isValid()
                    && gps.speed.isValid()
                    && gps.course.isValid()
                    && gps.date.isValid()
                    && gps.time.isValid()
                    && hasPositionFix()
                    ) {
                receivedData = true;
                displayDebugInfo();
                break;
            }
        }
    }

    // Mark as initialized
    gpsInitialized = true;
    lastReadingTime = millis();
    status = SensorStatus::OK;

    Serial.println("L76KB: Initialization complete");
    return status;
}

SensorStatus L76KBGPSSensor::update() {
    if (!gpsInitialized) {
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    // Read data from GPS module
    bool newData = false;
    unsigned long startTime = millis();

    // Read for up to 100ms or until buffer is empty
    while (serial.available() > 0 && (millis() - startTime < 100)) {
        char c = serial.read();
        gps.encode(c);
        newData = true;
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
    } else if (newData) {
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
        displayDebugInfo();
    }

    return status;
}

bool L76KBGPSSensor::isOperational() {
    return gpsInitialized && (status == SensorStatus::OK) && (millis() - lastDataTime < 5000);
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
    return gpsData.valid;
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

    // Reconfigure the module
    setUpdateRate(1); // 1Hz update rate
    sendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

    return true;
}

bool L76KBGPSSensor::sendCommand(const char* cmd) {
    serial.print(cmd);
    Serial.print("L76KB: Sent command: ");
    Serial.print(cmd);
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

String L76KBGPSSensor::getHDOPDescription(float hdopValue) const {
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

void L76KBGPSSensor::displayDebugInfo() {
    Serial.println("\n----- L76KB GPS Status -----");

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