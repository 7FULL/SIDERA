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
//    Serial.println("Initializing BMP388...");
//    if (!sensor.begin_I2C(address, &wire)) {
////        Serial.println("BMP388 initialization failed!");
//        status = SensorStatus::INITIALIZATION_ERROR;
//        return status;
//    }
//
//    // Set up sensor with default settings
//    sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
//    sensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
//    sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
//    sensor.setOutputDataRate(BMP3_ODR_50_HZ);

    // BMP388 (0x76 ó 0x77)
    if (!sensor.begin_I2C(address, &wire)) {
        Serial.println("¡No se encontró el sensor BMP388! Revisa conexiones y direcciones.");
        while (1) delay(10);
    }

    // Configuración del BMP388
    sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    sensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    sensor.setOutputDataRate(BMP3_ODR_50_HZ);

    status = SensorStatus::OK;
    Serial.println("BMP388 initialized successfully!");
    return status;
}

SensorStatus BMP388Sensor::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }

    if (!sensor.performReading()) {
        status = SensorStatus::READING_ERROR;
        return status;
    }

    temperature = sensor.temperature;
    pressure = sensor.pressure / 100.0F; // Convert to hPa
    altitude = sensor.readAltitude(SEA_LEVEL_PRESSURE_HPA) - referenceAltitude;

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