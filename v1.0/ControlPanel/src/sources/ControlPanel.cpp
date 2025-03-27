#include "headers/ControlPanel.h"
#include <mutex>

std::mutex i2cMutex;
bool buzzerActive;
unsigned long buzzerStartTime;

ControlPanel::ControlPanel()
        : lcd(0x27, 16, 2)
        , cePin(4)
        , csPin(5)
        , radio(cePin, csPin)
        , readingAddress("00001")
        , writingAddress("00002")
        , pinLaunchPlatform(35)
        , pinWakeup(32)
        , pinCountdown(34)
        , pinStopLaunch(27)
        , pinEmergencyStop(26)
        , countdownMax(10)
        , buzzerPin(17)
{}

void ControlPanel::begin() {
    pinMode(pinLaunchPlatform, INPUT);
    pinMode(pinWakeup, INPUT);
    pinMode(pinCountdown, INPUT);
    pinMode(pinStopLaunch, INPUT);
    pinMode(pinEmergencyStop, INPUT);

    Wire.begin(21, 22);
    delay(100);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("Control Panel");

    Serial.println("Control Panel initialized");

    radio.begin();

    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(11);
    radio.setRetries(3, 5);
    radio.setCRCLength(RF24_CRC_16);

    radio.openWritingPipe(00001);
    radio.openReadingPipe(1, 00002);

    //TODO A saber porque cojones no funciona el ack
    radio.setAutoAck(false);

    if (!radio.isChipConnected()) {
        Serial.println("¡Error en NRF24L01!");
        lcd.clear();
        lcd.print("Radio Error");
    } else {
        Serial.println("NRF24L01 OK");
    }

    radio.stopListening();
}

bool ControlPanel::isLaunchPlatformInitiated() const {
    return digitalRead(pinLaunchPlatform) == HIGH;
}

bool ControlPanel::isWakeupInitiated() const {
    return digitalRead(pinWakeup) == HIGH;
}

bool ControlPanel::isCountdownInitiated() const {
    return digitalRead(pinCountdown) == HIGH;
}

bool ControlPanel::isLaunchStopped() const {
    return digitalRead(pinStopLaunch) == HIGH;
}

bool ControlPanel::isEmergencyStop() const {
    return digitalRead(pinEmergencyStop) == HIGH;
}

void ControlPanel::updateDisplay(const char* message) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message);
}

void ControlPanel::updateDisplay(const char* message1, const char* message2) {
    std::lock_guard<std::mutex> lock(i2cMutex);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print(message2);
}

bool ControlPanel::startPlatform() {
    radio.stopListening();

    CommandData command;
    command.command = CMD_PING;  // Or create a specific platform start command
    command.parameter = 0;

    bool success = radio.write(&command, sizeof(CommandData));

    if (success) {
        Serial.println("Platform start command sent successfully");
    } else {
        Serial.println("Failed to send platform start command");
    }

//    radio.startListening();

    return success;
}

void ControlPanel::stopLaunch() {
    radio.stopListening();

    CommandData command;
    command.command = CMD_ABORT;
    command.parameter = 0;

    bool success = radio.write(&command, sizeof(CommandData));

    if (success) {
        Serial.println("Stop launch command sent successfully");
    } else {
        Serial.println("Failed to send stop launch command");
    }

    radio.startListening();
}

void ControlPanel::emergencyStop() {
    static uint32_t lastSendTime = 0;
    const uint32_t retryInterval = 10;

    if (millis() - lastSendTime >= retryInterval) {
        radio.stopListening();

        CommandData command;
        command.command = CMD_ABORT;
        command.parameter = 0;

        radio.write(&command, sizeof(CommandData));
        lastSendTime = millis();

        radio.startListening();
    }

    if (!buzzerActive) {
        tone(buzzerPin, 1000);
        buzzerStartTime = millis();
        buzzerActive = true;
    }
}

void ControlPanel::receiveCommands() {
    radio.startListening();

    // Check for incoming data without blocking
    if (radio.available()) {
        // Read command data
        CommandData receivedCommand;
        radio.read(&receivedCommand, sizeof(CommandData));

        Serial.println("[INFO] Command received: " + String(receivedCommand.command));
        // Process the command
        if (processCommand(receivedCommand)) {
            Serial.println("[INFO] Command processed: " + String(receivedCommand.command));
        } else {
            Serial.println("[WARNING] Invalid command received: " + String(receivedCommand.command));
        }
    }

//    radio.stopListening();
}

bool ControlPanel::processCommand(CommandData& command) {
    switch (command.command) {
        case CMD_PING:
            // Acknowledge ping (no specific action needed)
            Serial.println("[INFO] Ping received from control panel");
            return true;

        case CMD_ROCKET_READY:
            // Lunch the rocket
            Serial.println("[INFO] Lunch command received from control panel");
            rocketReady = true;
            return true;

        default:
            // Unknown command
            return false;
    }
}

bool ControlPanel::isRocketReady() {
    return rocketReady;
}

bool ControlPanel::launchRocket() {
    radio.stopListening();

    CommandData command;
    command.command = CMD_LAUNCH;
    command.parameter = 0;

    bool success = radio.write(&command, sizeof(CommandData));

    if (success) {
        Serial.println("Launch command sent successfully");
        updateDisplay("Launch cmd sent");
    } else {
        Serial.println("Failed to send launch command");
        updateDisplay("Launch failed");
    }

    radio.startListening();
    return success;
}

void ControlPanel::receiveTelemetry() {
    radio.startListening();

    if (radio.available()) {
        // Define a telemetry structure that matches the rocket's telemetry data
        TelemetryData telemetryData;

        // Read the telemetry data
        radio.read(&telemetryData, sizeof(TelemetryData));

        // Log telemetry
        Serial.print("Altitude: "); Serial.print(telemetryData.altitude); Serial.println("m");
        Serial.print("Temperature: "); Serial.print(telemetryData.temperature); Serial.println("°C");
        Serial.print("Pressure: "); Serial.print(telemetryData.pressure); Serial.println("hPa");
        Serial.print("Acceleration X: "); Serial.print(telemetryData.accelerationX); Serial.println("m/s^2");
        Serial.print("Acceleration Y: "); Serial.print(telemetryData.accelerationY); Serial.println("m/s^2");
        Serial.print("Acceleration Z: "); Serial.print(telemetryData.accelerationZ); Serial.println("m/s^2");
        Serial.print("Gyro X: "); Serial.print(telemetryData.gyroX); Serial.println("deg/s");
        Serial.print("Gyro Y: "); Serial.print(telemetryData.gyroY); Serial.println("deg/s");
        Serial.print("Gyro Z: "); Serial.print(telemetryData.gyroZ); Serial.println("deg/s");
        Serial.print("Latitude: "); Serial.println(telemetryData.latitude);
        Serial.print("Longitude: "); Serial.println(telemetryData.longitude);
        Serial.print("Parachute deployed: "); Serial.println(telemetryData.parachuteDeployed);
        Serial.print("Has reached apogee: "); Serial.println(telemetryData.hasReachedApogee);
        Serial.print("Timestamp: "); Serial.println(telemetryData.timestamp);

        if (telemetryData.landed) {
            Serial.println("Rocket has landed");
            rocketLanded = true;
            buzzerActive = true;
            buzzerStartTime = millis();

            // Update display with rocket location;
            char buffer[17]; // For 16x2 LCD
            char buffer2[17]; // For 16x2 LCD
            snprintf(buffer, sizeof(buffer), "Lat: %.6f", telemetryData.latitude);
            snprintf(buffer, sizeof(buffer), "Lon: %.6f", telemetryData.longitude);
            updateDisplay(buffer, buffer2);
        }else{
            // Update display with relevant telemetry (Altitude and acceleration)
            char buffer[17]; // For 16x2 LCD
            char buffer2[17]; // For 16x2 LCD
            snprintf(buffer, sizeof(buffer), "Alt: %.1fm", telemetryData.altitude);
            snprintf(buffer, sizeof(buffer2), "AX: %.1f", telemetryData.accelerationY);
            updateDisplay(buffer, buffer2);
        }
    }
}

void ControlPanel::updateBuzzer() {
    if (buzzerActive && (millis() - buzzerStartTime >= 1000)) {
        noTone(buzzerPin);
        buzzerActive = false;
    }
}

bool ControlPanel::countdown() {
    static unsigned long lastUpdate = 0;
    static int remaining = countdownMax;
    static char message[17];

    if (isEmergencyStop()) {
        return false;
    }

    if (millis() - lastUpdate >= 1000) {
        remaining--;
        snprintf(message, sizeof(message), "Countdown: %d", remaining);
        updateDisplay(message);
        lastUpdate = millis();
    }

    if (remaining == 0) {
        remaining = countdownMax;
        return true;
    }

    return false;
}

bool ControlPanel::wakeUp() {
    radio.stopListening();

    CommandData command;
    command.command = CMD_WAKE_UP;  // Or create a specific platform start command
    command.parameter = 0;

    bool success = radio.write(&command, sizeof(CommandData));

    if (success) {
        Serial.println("Platform start command sent successfully");
    } else {
        Serial.println("Failed to send platform start command");
    }

//    radio.startListening();
    return success;
}

bool ControlPanel::hasRocketLanded() {
    return rocketLanded;
}
