#include "SpecificTests.h"

BarometricSensorTest::BarometricSensorTest(BarometricSensorManager* manager)
        : manager(manager) {
}

TestResult BarometricSensorTest::runTest() {
    if (!manager) {
        return createResult(false, "Barometric sensor manager is null", 1001);
    }

    // Update sensors to get fresh readings
    manager->update();

    // Check if we have any operational sensors
    int operationalCount = manager->getOperationalSensorCount();
    if (operationalCount == 0) {
        return createResult(false, "No operational barometric sensors", 1002);
    }

    // Check altitude readings
    float altitude = manager->getAltitude();

    // Verify altitude is within a reasonable range for ground (±1000m from sea level)
    if (altitude < -1000.0f || altitude > 1000.0f) {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Altitude out of reasonable range: %.2f m", altitude);
        return createResult(false, errorMsg, 1003, altitude, 0.0f, 1000.0f);
    }

    // Check pressure readings
    float pressure = manager->getPressure();

    // Verify pressure is within a reasonable range (800-1100 hPa)
    if (pressure < 800.0f || pressure > 1100.0f) {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Pressure out of reasonable range: %.2f hPa", pressure);
        return createResult(false, errorMsg, 1004, pressure, 1013.25f, 100.0f);
    }

    Serial.println("---------------- Barometric sensor test passed ----------------");

    return createResult(true, "", 0, altitude);
}

String BarometricSensorTest::getName() const {
    return "BarometricSensorTest";
}

String BarometricSensorTest::getDescription() const {
    return "Tests barometric sensors for altitude and pressure";
}

bool BarometricSensorTest::isCritical() const {
    return true;  // Critical for flight
}

String BarometricSensorTest::getSubsystem() const {
    return "BAROMETER";
}

//
// IMU Sensor Test
//

IMUSensorTest::IMUSensorTest(IMUSensorManager* manager)
        : manager(manager) {
}

TestResult IMUSensorTest::runTest() {
    if (!manager) {
        return createResult(false, "IMU manager is null", 2001);
    }

    // Update sensors to get fresh readings
    manager->update();

    // Check if we have any operational sensors
    int operationalCount = manager->getOperationalSensorCount();
    if (operationalCount == 0) {
        Serial.println("No operational IMU sensors");
        return createResult(false, "No operational IMU sensors", 2002);
    }

    // Check accelerometer readings
    AccelerometerData accelData = manager->getAccelerometerData();

    // Verify acceleration magnitude is close to 1g (9.8 m/s²) when stationary
    if (accelData.magnitude < 8.0f || accelData.magnitude > 11.0f) {
        char errorMsg[64];
        Serial.println("Failure in accelerometer readings");
        Serial.printf("Accel magnitude: %.2f m/s²\n", accelData.magnitude);
        Serial.printf("Accel readings: x=%.2f, y=%.2f, z=%.2f\n", accelData.x, accelData.y, accelData.z);
        snprintf(errorMsg, sizeof(errorMsg), "Acceleration not near 1g: %.2f m/s²",
                 accelData.magnitude);
        return createResult(false, errorMsg, 2003, accelData.magnitude, 9.81f, 1.5f);
    }

    // Check gyroscope if available
    if (manager->hasGyroscope()) {
        GyroscopeData gyroData = manager->getGyroscopeData();

        // Verify gyro readings are near zero when stationary
        float gyroMagnitude = sqrt(gyroData.x*gyroData.x +
                                   gyroData.y*gyroData.y +
                                   gyroData.z*gyroData.z);

        if (gyroMagnitude > 15.0f) {  // Allow some noise, but not too much
            char errorMsg[64];
            Serial.println("Failure in gyro readings");
            Serial.printf("Gyro magnitude: %.2f deg/s\n", gyroMagnitude);
            Serial.printf("Gyro readings: x=%.2f, y=%.2f, z=%.2f\n", gyroData.x, gyroData.y, gyroData.z);
            snprintf(errorMsg, sizeof(errorMsg), "Gyro not near zero when stationary: %.2f deg/s",
                     gyroMagnitude);
            return createResult(false, errorMsg, 2004, gyroMagnitude, 0.0f, 15.0f);
        }
    }

    Serial.println("---------------- IMU sensor test passed ----------------");

    return createResult(true);
}

String IMUSensorTest::getName() const {
    return "IMUSensorTest";
}

String IMUSensorTest::getDescription() const {
    return "Tests IMU sensors for acceleration and rotation";
}

bool IMUSensorTest::isCritical() const {
    return true;  // Critical for flight
}

String IMUSensorTest::getSubsystem() const {
    return "IMU";
}

//
// GPS Sensor Test
//

GPSSensorTest::GPSSensorTest(GPSSensorManager* manager)
        : manager(manager) {
}

TestResult GPSSensorTest::runTest() {
    if (!manager) {
        return createResult(false, "GPS manager is null", 3001);
    }

    // Update GPS to get fresh readings
    manager->update();

    // Check if we have any operational sensors
    int operationalCount = manager->getOperationalSensorCount();
    if (operationalCount == 0) {
        return createResult(false, "No operational GPS sensors", 3002);
    }

    // Check if we have a position fix
    // This might fail indoors, so it's not necessarily a critical error
    if (!manager->hasPositionFix()) {
        Serial.println("No GPS position fix available");
        return createResult(false, "No GPS position fix", 3003);
    }

    // Get GPS data
    GPSData gpsData = manager->getGPSData();

    // Check satellite count (at least 4 for a 3D fix)
    if (gpsData.satellites < 4) {
        Serial.println("Insufficient satellites for GPS fix");
        Serial.printf("Satellites: %d (need at least 4)\n", gpsData.satellites);
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Insufficient satellites: %d (need ≥4)",
                 gpsData.satellites);
        return createResult(false, errorMsg, 3004, gpsData.satellites, 4.0f, 0.0f);
    }

    // Verify coordinates are valid
    if (gpsData.latitude == 0.0f && gpsData.longitude == 0.0f) {
        Serial.println("Invalid GPS coordinates");
        Serial.printf("Latitude: %.6f, Longitude: %.6f\n", gpsData.latitude, gpsData.longitude);
        return createResult(false, "Invalid GPS coordinates", 3005);
    }

    Serial.println("---------------- GPS sensor test passed ----------------");

    return createResult(true, "", 0, gpsData.satellites);
}

String GPSSensorTest::getName() const {
    return "GPSSensorTest";
}

String GPSSensorTest::getDescription() const {
    return "Tests GPS reception and position fix";
}

bool GPSSensorTest::isCritical() const {
    return false;  // Nice to have but not critical for flight
}

String GPSSensorTest::getSubsystem() const {
    return "GPS";
}

//
// LoRa Communication Test
//

LoRaCommunicationTest::LoRaCommunicationTest(LoRaSystem* loraSystem)
        : loraSystem(loraSystem) {
}

TestResult LoRaCommunicationTest::runTest() {
    if (!loraSystem) {
        return createResult(false, "LoRa system is null", 4001);
    }

    // Check if LoRa is operational
    if (!loraSystem->isOperational()) {
        return createResult(false, "LoRa system not operational", 4002);
    }

    // Send a test message
    Message testMessage;
    testMessage.type = MessageType::STATUS;
    testMessage.priority = 255;  // High priority
    testMessage.timestamp = millis();

    // Create a simple test payload
    uint8_t testData[] = "DIAGNOSTIC_TEST";
    testMessage.data = testData;
    testMessage.length = 14;

    // Send the message
    bool sendResult = loraSystem->sendMessage(testMessage);
    if (!sendResult) {
        return createResult(false, "Failed to send test message", 4003);
    }

    Serial.println("---------------- LoRa communication test passed ----------------");

    return createResult(true);
}

String LoRaCommunicationTest::getName() const {
    return "LoRaCommunicationTest";
}

String LoRaCommunicationTest::getDescription() const {
    return "Tests LoRa communication system";
}

bool LoRaCommunicationTest::isCritical() const {
    return true;  // Critical for telemetry and recovery
}

String LoRaCommunicationTest::getSubsystem() const {
    return "COMMUNICATION";
}

//
// Storage Test
//

StorageTest::StorageTest(StorageManager* manager)
        : manager(manager) {
}

TestResult StorageTest::runTest() {
    if (!manager) {
        return createResult(false, "Storage manager is null", 5001);
    }

    // Check if storage is operational
    if (!manager->isOperational()) {
        return createResult(false, "Storage system not operational", 5002);
    }

    // Check available space
    uint32_t availableSpace = manager->getAvailableSpace();
    if (availableSpace < 1024 * 100) {  // At least 100KB free
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Insufficient storage space: %lu bytes",
                 availableSpace);
        return createResult(false, errorMsg, 5003, availableSpace, 1024.0f * 100.0f, 0.0f);
    }

    // Test logging
    bool logResult = manager->logMessage(LogLevel::DEBUG, Subsystem::SYSTEM, "Storage diagnostic test");
    if (!logResult) {
        return createResult(false, "Failed to write test log", 5004);
    }

    // Test telemetry storage
    StoredTelemetry testTelemetry = {0};
    testTelemetry.timestamp = millis();
    testTelemetry.state = 0;  // INIT state
    bool telemetryResult = manager->storeTelemetry(testTelemetry);
    if (!telemetryResult) {
        return createResult(false, "Failed to store test telemetry", 5005);
    }

    // Force a flush to ensure data is written
    bool flushResult = manager->flush();
    if (!flushResult) {
        return createResult(false, "Failed to flush storage buffers", 5006);
    }

    Serial.println("---------------- Storage test passed ----------------");

    return createResult(true, "", 0, availableSpace);
}

String StorageTest::getName() const {
    return "StorageTest";
}

String StorageTest::getDescription() const {
    return "Tests storage system for reading and writing";
}

bool StorageTest::isCritical() const {
    return true;  // Critical for flight data
}

String StorageTest::getSubsystem() const {
    return "STORAGE";
}

//
// Battery Test
//

BatteryTest::BatteryTest(PowerManager* manager)
        : manager(manager) {
}

TestResult BatteryTest::runTest() {
    if (!manager) {
        return createResult(false, "Power manager is null", 6001);
    }

    // Update power readings
    manager->update();

    // Check battery voltage
    float voltage = manager->getBatteryVoltage();
    if (voltage < 3.3f) {  // Minimum acceptable voltage
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Battery voltage too low: %.2fV", voltage);
        return createResult(false, errorMsg, 6002, voltage, 3.7f, 0.4f);
    }

    // Check battery percentage
    int percentage = manager->getBatteryPercentage();
    if (percentage < 30) {  // At least 30% battery
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Battery level too low: %d%%", percentage);
        return createResult(false, errorMsg, 6003, percentage, 50.0f, 20.0f);
    }

    // Check power state
    PowerState state = manager->getPowerState();
    if (state != PowerState::NORMAL) {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Power state not normal: %d",
                 static_cast<int>(state));
        return createResult(false, errorMsg, 6004, static_cast<int>(state), 0.0f, 0.0f);
    }

    Serial.println("---------------- Battery test passed ----------------");

    return createResult(true, "", 0, voltage);
}

String BatteryTest::getName() const {
    return "BatteryTest";
}

String BatteryTest::getDescription() const {
    return "Tests battery voltage and capacity";
}

bool BatteryTest::isCritical() const {
    return true;  // Critical for flight
}

String BatteryTest::getSubsystem() const {
    return "POWER";
}

//
// Temperature Sensor Test
//

TemperatureSensorTest::TemperatureSensorTest(TemperatureSensorManager* manager)
        : manager(manager) {
}

TestResult TemperatureSensorTest::runTest() {
    if (!manager) {
        return createResult(false, "Temperature sensor manager is null", 6001);
    }

    // Update sensors to get fresh readings
    manager->update();

    // Check if we have any operational sensors
    int operationalCount = manager->getOperationalSensorCount();
    if (operationalCount == 0) {
        return createResult(false, "No operational temperature sensors", 6002);
    }

    // Check temperature readings
    float temperature = manager->getTemperature();

    // Verify temperature is within a reasonable range (-40°C to 85°C for DS18B20)
    if (temperature < -40.0f || temperature > 85.0f) {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "Temperature out of reasonable range: %.2f C", temperature);
        return createResult(false, errorMsg, 6003, temperature, 25.0f, 35.0f);
    }

    Serial.println("Temperature sensor readings:");
    Serial.printf("Temperature: %.2f C\n", temperature);
    Serial.printf("Operational sensors: %d\n", operationalCount);
    Serial.println("---------------- Temperature sensor test passed ----------------");

    return createResult(true, "", 0, temperature);
}

String TemperatureSensorTest::getName() const {
    return "TemperatureSensorTest";
}

String TemperatureSensorTest::getDescription() const {
    return "Tests temperature sensors for proper operation";
}

bool TemperatureSensorTest::isCritical() const {
    return false;  // Temperature is important but not critical for flight
}

String TemperatureSensorTest::getSubsystem() const {
    return "TEMPERATURE";
}