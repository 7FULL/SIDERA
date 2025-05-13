/**
 * DS18B20 Temperature Sensor Implementation
 */

#include "DS18B20Sensor.h"

DS18B20Sensor::DS18B20Sensor(uint8_t pin, uint8_t resolution)
        : pin(pin), resolution(resolution), temperature(0.0f), deviceFound(false),
          oneWire(pin), sensors(&oneWire) {
}

DS18B20Sensor::~DS18B20Sensor() {
    // No need for special cleanup
}

SensorStatus DS18B20Sensor::begin() {
    // Initialize the sensor
    sensors.begin();

    // Check if any devices are present
    if (sensors.getDeviceCount() == 0) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    // Get the address of the first device
    if (!sensors.getAddress(deviceAddress, 0)) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    deviceFound = true;

    // Set resolution
    sensors.setResolution(deviceAddress, resolution);

    status = SensorStatus::OK;
    return status;
}

void DS18B20Sensor::setAsyncResolution(uint8_t resolution) {
    this->resolution = resolution;

    // Set conversion delay based on resolution
    switch (resolution) {
        case 9:  conversionDelay = 94; break;
        case 10: conversionDelay = 188; break;
        case 11: conversionDelay = 375; break;
        case 12: conversionDelay = 750; break;
        default: conversionDelay = 750; break;
    }

    if (deviceFound) {
        sensors.setResolution(deviceAddress, resolution);
    }
}

SensorStatus DS18B20Sensor::update() {
    if (!deviceFound) {
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    unsigned long currentTime = millis();

    if (!conversionInProgress) {
        // Start new conversion
        sensors.requestTemperatures();
        conversionInProgress = true;
        conversionStartTime = currentTime;

        // Don't wait - return immediately
        return status;
    }

    // Check if conversion is complete
    if (currentTime - conversionStartTime >= conversionDelay) {
        // Read temperature from the device
        float temp = sensors.getTempC(deviceAddress);

        // Check if the read was successful
        if (temp == DEVICE_DISCONNECTED_C) {
            status = SensorStatus::READING_ERROR;
            conversionInProgress = false;
            return status;
        }

        // Apply offset
        temperature = temp + temperatureOffset;

#ifdef ENABLE_DS18B20_DEBUG
        Serial.print("DS18B20: Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");
#endif

        lastReadingTime = millis();
        status = SensorStatus::OK;
        conversionInProgress = false;
    }

    // If conversion not complete yet, just return current status
    return status;
}

bool DS18B20Sensor::isOperational() {
    // If it's been more than 5 seconds since last successful read, check again
    if (millis() - lastReadingTime > 5000 && status == SensorStatus::OK) {
        update();
    }

    return deviceFound && status == SensorStatus::OK;
}

SensorStatus DS18B20Sensor::getStatus() const {
    return status;
}

const char* DS18B20Sensor::getName() const {
    return "DS18B20";
}

unsigned long DS18B20Sensor::getLastReadingTime() const {
    return lastReadingTime;
}

float DS18B20Sensor::getTemperature() {
    return temperature;
}

void DS18B20Sensor::setTemperatureOffset(float offset) {
    temperatureOffset = offset;
}

float DS18B20Sensor::getTemperatureOffset() const {
    return temperatureOffset;
}

void DS18B20Sensor::setResolution(uint8_t resolution) {
    if (deviceFound) {
        this->resolution = resolution;
        sensors.setResolution(deviceAddress, resolution);
    }
}

uint8_t DS18B20Sensor::getResolution() const {
    return resolution;
}

DeviceAddress* DS18B20Sensor::getDeviceAddress() {
    return &deviceAddress;
}