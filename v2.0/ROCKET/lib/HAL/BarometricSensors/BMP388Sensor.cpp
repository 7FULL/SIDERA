/**
 * BMP388 Barometric Sensor Implementation
 */

#include "BMP388Sensor.h"

BMP388Sensor::BMP388Sensor(TwoWire& wire, uint8_t address)
        : wire(wire), address(address), temperature(0.0f), pressure(0.0f), altitude(0.0f) {
}

BMP388Sensor::~BMP388Sensor() {
    // No need for special cleanup
}

SensorStatus BMP388Sensor::begin() {
    Serial.println("BMP388: Starting initialization...");
    Serial.print("BMP388: Using I2C address 0x");
    Serial.println(address, HEX);

    // Try to initialize the sensor
    if (!sensor.begin_I2C(address, &wire)) {
        Serial.println("BMP388: Initialization failed!");
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    Serial.println("BMP388: Setting up parameters...");

    // Set up sensor with default settings
    sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    sensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    sensor.setOutputDataRate(BMP3_ODR_50_HZ);

    //Make a couple of readings to stabilize the sensor
    const int stabilizationReadings = 5;
    for (int i = 0; i < stabilizationReadings; ++i) {
        if (!sensor.performReading()) {
            Serial.println("BMP388: Failed to perform initial reading!");
            status = SensorStatus::READING_ERROR;
            return status;
        }
        delay(100); // Wait a bit between readings
    }

    status = SensorStatus::OK;
    Serial.println("BMP388: Initialization successful!");
    return status;
}

SensorStatus BMP388Sensor::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }

    if (!sensor.performReading()) {
        status = SensorStatus::READING_ERROR;

        Serial.println("BMP388: Failed to perform reading!");
        return status;
    }

    temperature = sensor.temperature;
    pressure = sensor.pressure / 100.0F; // Convert to hPa
    altitude = sensor.readAltitude(SEA_LEVEL_PRESSURE_HPA) - referenceAltitude;

    Serial.print("BMP388: Pressure: ");
    Serial.print(pressure);
    Serial.print(" hPa, Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");

    lastReadingTime = millis();
    status = SensorStatus::OK;
    return status;
}

bool BMP388Sensor::isOperational() {
    // If it's been more than 1 second since last successful read, check again
    if (millis() - lastReadingTime > 1000 && status == SensorStatus::OK) {
        update();
    }

    return status == SensorStatus::OK;
}

SensorStatus BMP388Sensor::getStatus() const {
    return status;
}

const char* BMP388Sensor::getName() const {
    return "BMP388";
}

unsigned long BMP388Sensor::getLastReadingTime() const {
    return lastReadingTime;
}

float BMP388Sensor::getAltitude() {
    return altitude;
}

float BMP388Sensor::getPressure() {
    return pressure;
}

float BMP388Sensor::getTemperature() {
    return temperature;
}

void BMP388Sensor::setReferenceAltitude(float altitude) {
    this->referenceAltitude = altitude;
}

float BMP388Sensor::getReferenceAltitude() const {
    return referenceAltitude;
}