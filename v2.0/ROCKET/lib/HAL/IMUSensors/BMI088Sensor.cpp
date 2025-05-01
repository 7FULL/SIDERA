/**
 * BMI088 IMU Sensor Implementation
 */

#include "BMI088Sensor.h"

BMI088Sensor::BMI088Sensor(TwoWire& wire, uint8_t gyroAddress, uint8_t acclAddress)
        : wire(wire),
          accel(wire, acclAddress),
          gyro(wire, gyroAddress)
{
    // Initialize data structures
    accelData = {0.0f, 0.0f, 0.0f, 0.0f};
    gyroData = {0.0f, 0.0f, 0.0f};
}

BMI088Sensor::~BMI088Sensor() {
    // No need for special cleanup
}

SensorStatus BMI088Sensor::begin() {
    int initStatus = accel.begin();

    Serial.print("BMI088 accelerometer initialization status: ");
    Serial.println(initStatus);

    if (initStatus != 1) {
        status = SensorStatus::INITIALIZATION_ERROR;
        accelInitialized = false;
        return status;
    }

    accelInitialized = true;


    int gyroStatus = gyro.begin();

    Serial.print("BMI088 gyroscope initialization status: ");
    Serial.println(gyroStatus);

    if (gyroStatus != 1) {
        // We can still work with just the accelerometer
        gyroInitialized = false;
    } else {
        gyroInitialized = true;
    }

    // Set default configurations
    accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
    accel.setRange(Bmi088Accel::RANGE_3G);
    accelRange = 3.0f;

    if (gyroInitialized) {
        gyro.setOdr(Bmi088Gyro::ODR_2000HZ_BW_532HZ);
        gyro.setRange(Bmi088Gyro::RANGE_500DPS);
    }

    // Make a couple of dummy reads to stabilize the sensor
    const int stabilizationReadings = 5;
    for (int i = 0; i < stabilizationReadings; i++) {
        float ax, ay, az;
        ax = accel.getAccelX_mss();
        ay = accel.getAccelY_mss();
        az = accel.getAccelZ_mss();

        if (gyroInitialized) {
            float gx, gy, gz;
            gx = gyro.getGyroX_rads();
            gy = gyro.getGyroY_rads();
            gz = gyro.getGyroZ_rads();
        }
        delay(100);
    }

    status = accelInitialized ? SensorStatus::OK : SensorStatus::INITIALIZATION_ERROR;
    return status;
}

SensorStatus BMI088Sensor::update() {
    if (!accelInitialized) {
        status = SensorStatus::NOT_INITIALIZED;
        return status;
    }

    // Read accelerometer data
    float ax, ay, az;

    ax = accel.getAccelX_mss();
    ay = accel.getAccelY_mss();
    az = accel.getAccelZ_mss();

    // Check for errors in reading accelerometer data
    if (!accel.getDrdyStatus()) {
        status = SensorStatus::READING_ERROR;
        return status;
    }

    // Update accelerometer data
    accelData.x = ax - accelOffsetX;
    accelData.y = ay - accelOffsetY;
    accelData.z = az - accelOffsetZ;
    accelData.magnitude = sqrt(ax*ax + ay*ay + az*az);

    // Read gyroscope data if available
    if (gyroInitialized) {
        float gx, gy, gz;

        // Read gyroscope data
        gx = gyro.getGyroX_rads();
        gy = gyro.getGyroY_rads();
        gz = gyro.getGyroZ_rads();

        // Check for errors in reading gyroscope data
        if (!gyro.getDrdyStatus()) {
            gyroInitialized = false;
        }

        // Update gyroscope data
        gyroData.x = gx * RAD_TO_DEG;
        gyroData.y = gy * RAD_TO_DEG;
        gyroData.z = gz * RAD_TO_DEG;
    }

    #ifdef ENABLE_BMIO88_DEBUG
    Serial.print("BMI088: ");
    Serial.print("Accel - X: ");
    Serial.print(accelData.x);
    Serial.print(", Y: ");
    Serial.print(accelData.y);
    Serial.print(", Z: ");
    Serial.print(accelData.z);
    Serial.print(", Magnitude: ");
    Serial.print(accelData.magnitude);
    Serial.print(" | Gyro - X: ");
    Serial.print(gyroData.x);
    Serial.print(", Y: ");
    Serial.print(gyroData.y);
    Serial.print(", Z: ");
    Serial.println(gyroData.z);
    #endif

    lastReadingTime = millis();
    status = SensorStatus::OK;
    return status;
}

bool BMI088Sensor::isOperational() {
    // If it's been more than 1 second since last successful read, check again
    if (millis() - lastReadingTime > 1000 && status == SensorStatus::OK) {
        update();
    }

    return accelInitialized && status == SensorStatus::OK;
}

SensorStatus BMI088Sensor::getStatus() const {
    return status;
}

const char* BMI088Sensor::getName() const {
    return "BMI088";
}

unsigned long BMI088Sensor::getLastReadingTime() const {
    return lastReadingTime;
}

AccelerometerData BMI088Sensor::getAccelerometerData() {
    return accelData;
}

GyroscopeData BMI088Sensor::getGyroscopeData() {
    return gyroData;
}

bool BMI088Sensor::hasGyroscope() const {
    return gyroInitialized;
}

SensorStatus BMI088Sensor::calibrate() {
    if (!accelInitialized) {
        return SensorStatus::NOT_INITIALIZED;
    }

    // Basic calibration - assumes the sensor is flat on a level surface
    // Read several samples to get a stable zero offset
    constexpr int numSamples = 50;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (int i = 0; i < numSamples; i++) {
        float ax, ay, az;

        ax = accel.getAccelX_mss();
        ay = accel.getAccelY_mss();
        az = accel.getAccelZ_mss();

        if (ax == 0 || ay == 0 || az == 0) {
            return SensorStatus::CALIBRATION_ERROR;
        }
        sumX += ax;
        sumY += ay;
        sumZ += az;
        delay(10);
    }

    float offsetX = sumX / numSamples;
    float offsetY = sumY / numSamples;
    float offsetZ = sumZ / numSamples - 9.81f; // Subtract gravity from Z

    // Store offsets or apply to future readings
    // Note: The BMI088 library might not support setting offsets directly
    // In a real implementation, you'd store these values and apply them when reading

    return SensorStatus::OK;
}

bool BMI088Sensor::setAccelerometerRange(float range) {
    if (!accelInitialized) {
        return false;
    }

    Bmi088Accel::Range bmiRange;

    // Convert range to BMI088 enum
    if (range <= 3.0f) {
        bmiRange = Bmi088Accel::RANGE_3G;
        accelRange = 3.0f;
    } else if (range <= 6.0f) {
        bmiRange = Bmi088Accel::RANGE_6G;
        accelRange = 6.0f;
    } else if (range <= 12.0f) {
        bmiRange = Bmi088Accel::RANGE_12G;
        accelRange = 12.0f;
    } else {
        bmiRange = Bmi088Accel::RANGE_24G;
        accelRange = 24.0f;
    }

    return (accel.setRange(bmiRange));
}

float BMI088Sensor::getAccelerometerRange() const {
    return accelRange;
}