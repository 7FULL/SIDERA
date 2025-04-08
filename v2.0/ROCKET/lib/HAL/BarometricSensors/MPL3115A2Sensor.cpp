/**
 * MPL3115A2 Barometric Sensor Implementation
 */

#include "MPL3115A2Sensor.h"

MPL3115A2Sensor::MPL3115A2Sensor(TwoWire& wire)
        : wire(wire), temperature(0.0f), pressure(0.0f), altitude(0.0f) {
}

MPL3115A2Sensor::~MPL3115A2Sensor() {
    // No need for special cleanup
}

SensorStatus MPL3115A2Sensor::begin() {
    // Begin communication with the MPL3115A2 sensor
    if (!sensor.begin(&wire)) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    // Set to altimeter mode
    sensor.setMode(MPL3115A2_ALTIMETER);

    // TODO
    // Optional: set oversampling ratio for higher precision
    // 0-7 is valid, higher values = more samples
    // sensor.setOversampleRate(7);
    // sensor.setMode(MPL3115A2_ALTITUDE);
    status = SensorStatus::OK;
    return status;
}

SensorStatus MPL3115A2Sensor::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }

    // Read altitude data
    float newAltitude = sensor.getAltitude();

    // Check if the read was successful
    if (isnan(newAltitude)) {
        status = SensorStatus::READING_ERROR;
        return status;
    }

    // Read pressure data
    float newPressure = sensor.getPressure();

    // Check if the read was successful
    if (isnan(newPressure)) {
        status = SensorStatus::READING_ERROR;
        return status;
    }

    // Read temperature data
    float newTemperature = sensor.getTemperature();

    // Check if the read was successful
    if (isnan(newTemperature)) {
        status = SensorStatus::READING_ERROR;
        return status;
    }

    // Update sensor data
    altitude = newAltitude - referenceAltitude;
    pressure = newPressure; // Already in Pa
    temperature = newTemperature;

    lastReadingTime = millis();
    status = SensorStatus::OK;
    return status;
}

bool MPL3115A2Sensor::isOperational() {
    // If it's been more than 1 second since last successful read, check again
    if (millis() - lastReadingTime > 1000 && status == SensorStatus::OK) {
        update();
    }

    return status == SensorStatus::OK;
}

SensorStatus MPL3115A2Sensor::getStatus() const {
    return status;
}

const char* MPL3115A2Sensor::getName() const {
    return "MPL3115A2";
}

unsigned long MPL3115A2Sensor::getLastReadingTime() const {
    return lastReadingTime;
}

float MPL3115A2Sensor::getAltitude() {
    return altitude;
}

float MPL3115A2Sensor::getPressure() {
    return pressure;
}

float MPL3115A2Sensor::getTemperature() {
    return temperature;
}

void MPL3115A2Sensor::setReferenceAltitude(float altitude) {
    this->referenceAltitude = altitude;
}

float MPL3115A2Sensor::getReferenceAltitude() const {
    return referenceAltitude;
}