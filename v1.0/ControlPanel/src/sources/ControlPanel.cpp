#include "headers/ControlPanel.h"
#include "utils/Comm.h"
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
    radio.openReadingPipe(1, readingAddress);
    radio.openWritingPipe(writingAddress);
    radio.setPALevel(RF24_PA_MAX);
    radio.stopListening();

    if (!radio.isChipConnected()) {
        Serial.println("¡Error en NRF24L01!");
        lcd.clear();
        lcd.print("Radio Error");
    } else {
        Serial.println("NRF24L01 OK");
    }
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

void ControlPanel::startPlatform() {
    radio.stopListening();

    char message[] = "start_platform";

    char buffer[100];

    Comm::generateMessage(message, buffer);

    radio.write(buffer, sizeof(buffer));
    Serial.println("Message sent: " + String(buffer));

    delay(10);
}

void ControlPanel::stopLaunch() {
    radio.stopListening();

    char message[] = "stop_launch";

    char buffer[100];

    Comm::generateMessage(message, buffer);

    radio.write(buffer, sizeof(buffer));
    Serial.println("Message sent: " + String(buffer));

    delay(10);
}
void ControlPanel::emergencyStop() {
    static uint32_t lastSendTime = 0;
    const uint32_t retryInterval = 10;

    if (millis() - lastSendTime >= retryInterval) {
        radio.stopListening();
        char buffer[100];
        Comm::generateMessage("emergency_stop", buffer);
        radio.write(buffer, sizeof(buffer));
        lastSendTime = millis();
    }

    if (!buzzerActive) {
        tone(buzzerPin, 1000);
        buzzerStartTime = millis();
        buzzerActive = true;
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

    if (remaining == countdownMax) {
        radio.stopListening();
        char buffer[100];
        Comm::generateMessage("countdown_started", buffer);
        radio.write(buffer, sizeof(buffer));
        Serial.println("Message sent: " + String(buffer));

        remaining--;
        snprintf(message, sizeof(message), "Countdown: %d", remaining);
        updateDisplay(message);
        lastUpdate = millis();
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

    char message[] = "wakeup_rocket";

    char buffer[100];

    Comm::generateMessage(message, buffer);

    radio.write(buffer, sizeof(buffer));
    Serial.println("Message sent: " + String(buffer));

    delay(10);

    return true;
}
