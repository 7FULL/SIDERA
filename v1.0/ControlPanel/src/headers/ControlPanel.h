#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RF24.h"

class ControlPanel {
public:
    ControlPanel();

    void begin();

    bool isLaunchPlatformInitiated() const;
    bool isWakeupInitiated() const;
    bool isCountdownInitiated() const;
    bool isLaunchStopped() const;
    bool isEmergencyStop() const;

    void updateDisplay(const char* message);

    bool wakeUp();
    bool countdown();
    void startPlatform();
    void emergencyStop();

    void updateBuzzer();

private:
    const uint8_t pinLaunchPlatform;
    const uint8_t pinWakeup;
    const uint8_t pinCountdown;
    const uint8_t pinStopLaunch;
    const uint8_t pinEmergencyStop;

    const byte readingAddress[6];
    const byte writingAddress[6];
    const uint8_t cePin;
    const uint8_t csPin;

    const uint8_t buzzerPin;

    const byte countdownMax;

    LiquidCrystal_I2C lcd;
    RF24 radio; // CE, CSN

    void stopLaunch();
};

#endif
