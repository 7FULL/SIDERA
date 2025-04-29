/**
 * ADXL375 High-G Accelerometer Implementation
 */

#include "ADXL375Sensor.h"

ADXL375Sensor::ADXL375Sensor(TwoWire& wire, uint8_t address, int32_t sensorID)
        : wire(wire), sensor(sensorID, &wire), sensorID(sensorID), address(address) {
    // Initialize data structures
    accelData = {0.0f, 0.0f, 0.0f, 0.0f};
    dummyGyroData = {0.0f, 0.0f, 0.0f};
}

ADXL375Sensor::~ADXL375Sensor() {
    // No need for special cleanup
}

SensorStatus ADXL375Sensor::begin() {
    // Initialize the sensor - corregido para que coincida con la API
    if (!sensor.begin(address)) {
        status = SensorStatus::INITIALIZATION_ERROR;
        return status;
    }

    // Set default configurations
    sensor.setRange(ADXL34X_RANGE_16_G);
    sensor.setDataRate(ADXL3XX_DATARATE_100_HZ); // Usar el valor correcto del enum

    // Configurar detección de actividad usando la API disponible
    // Habilitamos actividad en XYZ
    sensor.writeRegister(ADXL3XX_REG_ACT_INACT_CTL, 0x70); // 0x70 = 0b01110000 (X,Y,Z enable)
    // Umbral de actividad ~20g (valor aprox, cada LSB ~0.063g)
    uint8_t threshold = 20 / 0.063;
    sensor.writeRegister(ADXL3XX_REG_THRESH_ACT, threshold);

    // La ADXL375 tiene un rango fijo de ±200g
    accelRange = 200.0f;

    // Make a couple of dummy reads to stabilize the sensor
    const int stabilizationReadings = 5;
    for (int i = 0; i < stabilizationReadings; i++) {
        sensors_event_t event;
        sensor.getEvent(&event);
        // Dummy read to stabilize the sensor
        accelData.x = event.acceleration.x;
        accelData.y = event.acceleration.y;
        accelData.z = event.acceleration.z;
        accelData.magnitude = sqrt(
                accelData.x * accelData.x +
                accelData.y * accelData.y +
                accelData.z * accelData.z
        );

        Serial.print("Stabilization read: ");
        Serial.print("X: ");
        Serial.print(accelData.x);
        Serial.print(", Y: ");
        Serial.print(accelData.y);
        Serial.print(", Z: ");
        Serial.print(accelData.z);
        Serial.print(", Magnitude: ");
        Serial.println(accelData.magnitude);
        delay(100);
    }

    status = SensorStatus::OK;
    return status;
}

SensorStatus ADXL375Sensor::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }

    Serial.println("Updating ADXL375 sensor...");

    // Get acceleration event
    sensors_event_t event;
    sensor.getEvent(&event);

    // Apply calibration offsets
    accelData.x = event.acceleration.x - offsetX;
    accelData.y = event.acceleration.y - offsetY;
    accelData.z = event.acceleration.z - offsetZ;
    accelData.magnitude = sqrt(
            accelData.x * accelData.x +
            accelData.y * accelData.y +
            accelData.z * accelData.z
    );

    lastReadingTime = millis();
    status = SensorStatus::OK;
    return status;
}

bool ADXL375Sensor::isOperational() {
    // If it's been more than 1 second since last successful read, check again
    if (millis() - lastReadingTime > 1000 && status == SensorStatus::OK) {
        update();
    }

    return status == SensorStatus::OK;
}

SensorStatus ADXL375Sensor::getStatus() const {
    return status;
}

const char* ADXL375Sensor::getName() const {
    return "ADXL375";
}

unsigned long ADXL375Sensor::getLastReadingTime() const {
    return lastReadingTime;
}

AccelerometerData ADXL375Sensor::getAccelerometerData() {
    return accelData;
}

GyroscopeData ADXL375Sensor::getGyroscopeData() {
    // ADXL375 has no gyroscope
    return dummyGyroData;
}

bool ADXL375Sensor::hasGyroscope() const {
    // ADXL375 has no gyroscope
    return false;
}

SensorStatus ADXL375Sensor::calibrate() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return SensorStatus::NOT_INITIALIZED;
    }

    // Basic calibration - assumes the sensor is flat on a level surface
    // Read several samples to get a stable zero offset
    constexpr int numSamples = 50;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (int i = 0; i < numSamples; i++) {
        sensors_event_t event;
        sensor.getEvent(&event);

        sumX += event.acceleration.x;
        sumY += event.acceleration.y;
        sumZ += event.acceleration.z;
        delay(10);
    }

    // Calculate average offsets
    offsetX = sumX / numSamples;
    offsetY = sumY / numSamples;
    offsetZ = sumZ / numSamples - 9.81f; // Subtract gravity from Z

    return SensorStatus::OK;
}

bool ADXL375Sensor::setAccelerometerRange(float range) {
    // ADXL375 has a fixed range of ±200g, so this is a no-op
    return false;
}

float ADXL375Sensor::getAccelerometerRange() const {
    return accelRange;
}

bool ADXL375Sensor::setDataRate(adxl3xx_dataRate_t dataRate) {
    sensor.setDataRate(dataRate);
    return true;
}