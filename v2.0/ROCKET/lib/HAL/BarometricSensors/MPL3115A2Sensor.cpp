/**
 * MPL3115A2 Barometric Sensor Implementation
 */

#include "MPL3115A2Sensor.h"

#define MPL3115A2_STATUS           0x00
#define MPL3115A2_OUT_P_MSB        0x01
#define MPL3115A2_OUT_P_CSB        0x02
#define MPL3115A2_OUT_P_LSB        0x03
#define MPL3115A2_OUT_T_MSB        0x04
#define MPL3115A2_OUT_T_LSB        0x05
#define MPL3115A2_DR_STATUS        0x06
#define MPL3115A2_WHO_AM_I         0x0C
#define MPL3115A2_CTRL_REG1        0x26
#define MPL3115A2_CTRL_REG2        0x27
#define MPL3115A2_CTRL_REG3        0x28
#define MPL3115A2_CTRL_REG4        0x29
#define MPL3115A2_CTRL_REG5        0x2A
#define MPL3115A2_PT_DATA_CFG      0x13

MPL3115A2Sensor::MPL3115A2Sensor(TwoWire& wire, uint8_t address)
        : wire(wire), address(address), temperature(0.0f), pressure(0.0f), altitude(0.0f) {
}

MPL3115A2Sensor::~MPL3115A2Sensor() {
    // No need for special cleanup
}

SensorStatus MPL3115A2Sensor::begin() {
    // Comprueba conexión con el sensor
    if (!sensor.begin(&wire)) {
        Serial.println("¡No se encontró el sensor MPL3115A2! Verifique las conexiones.");
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    status = SensorStatus::OK;
    return status;
}

SensorStatus MPL3115A2Sensor::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }
    float newAltitude = 0;
    float newPressure = 0;
    float newTemperature = 0;

    sensor.setMode(MPL3115A2_BAROMETER); // Asegúrate de que está en modo barómetro

    // Lecturas de presión y temperatura
    float pressure = sensor.getPressure();       // en Pascales (Pa)
    float temperature = sensor.getTemperature(); // en °C

    // Si está en modo barómetro, convertimos Pa a hPa
    float pressure_hpa = pressure / 100.0;

    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.print(" °C\t");
    Serial.print("Presión: ");
    Serial.print(pressure_hpa);
    Serial.println(" hPa");

    // Si quisieras leer altitud en modo altímetro:
    sensor.setMode(MPL3115A2_ALTIMETER);

    float altitude = sensor.getAltitude(); // en metros
    Serial.print("Altitud: ");
    Serial.print(altitude);
    Serial.println(" m");

    // Update sensor data
    altitude = newAltitude - referenceAltitude;
    pressure = newPressure; // Already in Pa
    temperature = newTemperature;

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" Pa, Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");

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