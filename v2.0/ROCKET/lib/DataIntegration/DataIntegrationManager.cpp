/**
 * Data Integration Manager Implementation
 */

#include "DataIntegrationManager.h"

DataIntegrationManager::DataIntegrationManager(
        BarometricSensorManager* baroManager,
        IMUSensorManager* imuManager,
        GPSSensorManager* gpsManager,
        TemperatureSensorManager* tempManager,
        PowerManager* powerManager
)
        : baroManager(baroManager),
          imuManager(imuManager),
          gpsManager(gpsManager),
          tempManager(tempManager),
          powerManager(powerManager),
          currentState(RocketState::INIT),
          apogeeDetected(false),
          maxAltitude(0.0f),
          descentCount(0),
          landingDetected(false),
          landingAltitude(0.0f),
          stableStartTime(0),
          stableAltitudeDetected(false),
          lastAltitude(0.0f),
          lastAltitudeTime(0)
{
    // Initialize flight data structure
    flightData = {0};
    flightData.timestamp = millis();
    flightData.state = RocketState::INIT;
}

bool DataIntegrationManager::begin() {
    // Initialize internal state
    apogeeDetected = false;
    maxAltitude = 0.0f;
    descentCount = 0;
    landingDetected = false;
    landingAltitude = 0.0f;
    stableStartTime = 0;
    stableAltitudeDetected = false;
    lastAltitude = 0.0f;
    lastAltitudeTime = 0;

    // Get initial readings
    if (baroManager) {
        flightData.altitude = baroManager->getAltitude();
        flightData.pressure = baroManager->getPressure();
        flightData.temperature = baroManager->getTemperature();
        lastAltitude = flightData.altitude;
    }

    if (imuManager) {
        flightData.accelData = imuManager->getAccelerometerData();
        flightData.verticalAccel = flightData.accelData.z; // Assuming Z is up
        if (imuManager->hasGyroscope()) {
            flightData.gyroData = imuManager->getGyroscopeData();
        }
    }

    if (gpsManager && gpsManager->hasPositionFix()) {
        flightData.gpsData = gpsManager->getGPSData();
        flightData.gpsSatellites = flightData.gpsData.satellites;
    }

    if (powerManager) {
        flightData.batteryVoltage = powerManager->getBatteryVoltage();
    }

    // Set initial confidence (placeholder)
    flightData.confidence = 1.0f;

    Serial.println("DataIntegrationManager initialized");
    return true;
}

void DataIntegrationManager::update() {
    unsigned long currentTime = millis();
    flightData.timestamp = currentTime;
    flightData.state = currentState;

    //Update sensor readings
    if (baroManager) {
        baroManager->update();
    }
    if (imuManager) {
        imuManager->update();
    }
    if (gpsManager) {
        gpsManager->update();
    }
    if (powerManager) {
        powerManager->update();
    }
    if (tempManager) {
        tempManager->update();
    }

    // Update sensor readings
    if (baroManager) {
        flightData.altitude = baroManager->getAltitude();
        flightData.pressure = baroManager->getPressure();
        flightData.temperature = baroManager->getTemperature();
    }

    if (imuManager) {
        flightData.accelData = imuManager->getAccelerometerData();
        flightData.verticalAccel = flightData.accelData.z; // Assuming Z is up
        if (imuManager->hasGyroscope()) {
            flightData.gyroData = imuManager->getGyroscopeData();
        }
    }

    if (gpsManager && gpsManager->hasPositionFix()) {
        flightData.gpsData = gpsManager->getGPSData();
        flightData.gpsSatellites = flightData.gpsData.satellites;
    }

    if (powerManager) {
        flightData.batteryVoltage = powerManager->getBatteryVoltage();
    }

    if (tempManager) {
        // Override temperature with dedicated sensor if available
        flightData.temperature = tempManager->getTemperature();
    }

    // Calculate vertical speed
    calculateVerticalSpeed();

    // Update apogee and landing detection
    detectApogee();
    detectLanding();

    // Update flight data flags
    flightData.apogeeDetected = apogeeDetected;
    flightData.landingDetected = landingDetected;

    // Placeholder for confidence calculation (simplified)
    flightData.confidence = 1.0f;

    // Update lastAltitude for next calculation
    lastAltitude = flightData.altitude;
    lastAltitudeTime = currentTime;
}

void DataIntegrationManager::setCurrentState(RocketState state) {
    currentState = state;
    flightData.state = state;
}

FlightData DataIntegrationManager::getFlightData() {
    return flightData;
}

bool DataIntegrationManager::isApogeeDetected() {
    return apogeeDetected;
}

bool DataIntegrationManager::isLandingDetected() {
    return landingDetected;
}

void DataIntegrationManager::calculateVerticalSpeed() {
    // Skip if we don't have previous altitude data
    if (lastAltitudeTime == 0) {
        flightData.verticalSpeed = 0.0f;
        return;
    }

    // Calculate delta time in seconds
    float deltaTime = (millis() - lastAltitudeTime) / 1000.0f;

    // Avoid division by zero
    if (deltaTime <= 0.0f) {
        return;
    }

    // Calculate vertical speed
    flightData.verticalSpeed = (flightData.altitude - lastAltitude) / deltaTime;
}

void DataIntegrationManager::detectApogee() {
    // Update max altitude
    if (flightData.altitude > maxAltitude) {
        maxAltitude = flightData.altitude;
        descentCount = 0;
        return;
    }

    // Check if we're significantly below max altitude for several consecutive readings
    if (maxAltitude - flightData.altitude > APOGEE_DETECTION_THRESHOLD) {
        descentCount++;

        if (descentCount >= APOGEE_DETECTION_WINDOW && !apogeeDetected) {
            apogeeDetected = true;

            Serial.print("Apogee detected at altitude: ");
            Serial.print(maxAltitude);
            Serial.println(" meters");
        }
    } else {
        // Reset descent counter if we're not consistently descending
        descentCount = 0;
    }
}

void DataIntegrationManager::detectLanding() {
    // First time initialization
    if (landingAltitude == 0.0f) {
        landingAltitude = flightData.altitude;
        return;
    }

    // Check if the altitude is stable
    if (abs(flightData.altitude - landingAltitude) < LANDED_ALTITUDE_THRESHOLD) {
        if (!stableAltitudeDetected) {
            stableAltitudeDetected = true;
            stableStartTime = millis();
        }

        // Check if we've been stable for long enough
        if (millis() - stableStartTime >= LANDED_STABILITY_TIME && !landingDetected) {
            landingDetected = true;

            Serial.print("Landing detected at altitude: ");
            Serial.print(flightData.altitude);
            Serial.println(" meters");
        }
    } else {
        // Update landing altitude and reset stable detection
        landingAltitude = flightData.altitude;
        stableAltitudeDetected = false;
    }
}