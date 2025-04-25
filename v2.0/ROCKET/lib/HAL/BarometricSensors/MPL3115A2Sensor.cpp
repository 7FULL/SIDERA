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

// Write a value to a register
void MPL3115A2Sensor::writeRegister(byte reg, byte value) {
    wire.beginTransmission(address);
    wire.write(reg);
    wire.write(value);
    wire.endTransmission();
}

// Read a value from a register
byte MPL3115A2Sensor::readRegister(byte reg) {
    wire.beginTransmission(address);
    wire.write(reg);
    wire.endTransmission(false);

    wire.requestFrom(address, 1);
    if (wire.available()) {
        return wire.read();
    }
    return 0;
}

SensorStatus MPL3115A2Sensor::begin() {
    wire.beginTransmission(address);
    wire.write(MPL3115A2_WHO_AM_I);
    wire.endTransmission(false);

    wire.requestFrom(address, 1);

    if (wire.available()) {
        byte whoAmI = wire.read();
        Serial.print("WHO_AM_I register value: 0x");
        Serial.println(whoAmI, HEX);

        if (whoAmI == 0xC4) {
            Serial.println("MPL3115A2 sensor detected!");
        } else {
            Serial.println("Unknown device detected!");
        }
    } else {
        Serial.println("No response from the sensor!");
    }

    Serial.println("Initializing MPL3115A2 sensor...");
    // Configure the sensor
    // Reset the sensor
    writeRegister(MPL3115A2_CTRL_REG1, 0x04); // Software reset
    delay(10);

    // Configure pressure and temperature measurement with oversampling ratio of 128
    writeRegister(MPL3115A2_CTRL_REG1, 0x38); // Set SBYB=0, OST=0, RST=0, ALT=0, RAW=0, OS=111 (oversample ratio=128)

    // Enable data flags
    writeRegister(MPL3115A2_PT_DATA_CFG, 0x07); // Enable pressure, temperature, and altitude data ready flags

    // Set to active mode and altimeter mode
    writeRegister(MPL3115A2_CTRL_REG1, 0xB9); // Set SBYB=1, OST=1, RST=0, ALT=1, RAW=0, OS=111 (oversample ratio=128)

    Serial.println("MPL3115A2 initialized successfully!");

    return status = SensorStatus::OK;
}

SensorStatus MPL3115A2Sensor::update() {
    if (status == SensorStatus::NOT_INITIALIZED) {
        return status;
    }
    float newAltitude = 0;
    float newPressure = 0;
    float newTemperature = 0;

    // Read data from the sensor
    // Toggle OST bit to initiate a measurement
    byte ctrl_reg1 = readRegister(MPL3115A2_CTRL_REG1);
    writeRegister(MPL3115A2_CTRL_REG1, ctrl_reg1 | 0x02);  // Set OST bit

    // Wait for the measurement to complete
    delay(10);

    // Check if data is ready
    if (readRegister(MPL3115A2_STATUS) & 0x08) {  // Check PDR (Pressure Data Ready) bit
        // Read altitude
        wire.beginTransmission(address);
        wire.write(MPL3115A2_OUT_P_MSB);  // Start from pressure MSB
        wire.endTransmission(false);

        wire.requestFrom(address, 5);  // Request 5 bytes (pressure + temperature)

        uint32_t pressure_raw = 0;
        if (wire.available() >= 3) {
            pressure_raw = wire.read() << 16;  // MSB
            pressure_raw |= wire.read() << 8;   // CSB
            pressure_raw |= wire.read();        // LSB
        }

        // The pressure is in Q18.2 format (18 integer bits and 2 fractional bits)
        newPressure = (float)pressure_raw / 4.0;  // Convert to Pa

        // Check if the read was successful
        if (isnan(newPressure)) {
            status = SensorStatus::READING_ERROR;
            return status;
        }

        // Read temperature
        int16_t temp_raw = 0;
        if (wire.available() >= 2) {
            temp_raw = wire.read() << 8;  // MSB
            temp_raw |= wire.read();      // LSB
        }

        // The temperature is in Q8.4 format (8 integer bits and 4 fractional bits)
        newTemperature = (float)temp_raw / 256.0;  // Convert to °C

        // Check if the read was successful
        if (isnan(newTemperature)) {
            status = SensorStatus::READING_ERROR;
            return status;
        }

        // Switch to altimeter mode to read altitude
        writeRegister(MPL3115A2_CTRL_REG1, (ctrl_reg1 | 0x80));  // Set ALT bit
        delay(10);

        // Toggle OST bit to initiate an altitude measurement
        writeRegister(MPL3115A2_CTRL_REG1, (ctrl_reg1 | 0x82));  // Set OST and ALT bits
        delay(10);

        // Read altitude
        wire.beginTransmission(address);
        wire.write(MPL3115A2_OUT_P_MSB);
        wire.endTransmission(false);

        wire.requestFrom(address, 3);

        uint32_t altitude_raw = 0;
        if (wire.available() >= 3) {
            altitude_raw = wire.read() << 16;
            altitude_raw |= wire.read() << 8;
            altitude_raw |= wire.read();
        }

        // The altitude is in Q16.4 format (16 integer bits and 4 fractional bits)
        uint32_t alt20 = altitude_raw >> 4;           // Q16.4
        newAltitude = alt20 / 16.0;             // metros

        // Check if the read was successful
        if (isnan(newAltitude)) {
            status = SensorStatus::READING_ERROR;
            return status;
        }

        // Switch back to barometer mode
        writeRegister(MPL3115A2_CTRL_REG1, ctrl_reg1 & ~0x80);  // Clear ALT bit
    } else {
        Serial.println("Waiting for data...");
    }

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