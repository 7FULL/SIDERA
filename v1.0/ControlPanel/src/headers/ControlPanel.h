#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RF24.h"

enum CommandCodes {
    CMD_PING = 1,
    CMD_DEPLOY_PARACHUTE = 2,
    CMD_ABORT = 3,
    CMD_REBOOT = 4,
    CMD_WAKE_UP = 5,
    CMD_LAUNCH = 6,
    CMD_START_PLATFORM = 7,
    CMD_ROCKET_READY = 8,
};

struct TelemetryData {
    float altitude;
    float temperature;
    float pressure;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float latitude;
    float longitude;
    bool parachuteDeployed;
    bool hasReachedApogee;
    uint32_t timestamp;
};

struct CommandData {
    uint8_t command;
    uint32_t parameter;
};

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

    bool launchRocket();

    bool receiveTelemetry();

    void receiveCommands();

    bool isRocketReady();

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

    bool rocketReady;

    const uint8_t buzzerPin;

    const byte countdownMax;

    LiquidCrystal_I2C lcd;
    RF24 radio; // CE, CSN

    void stopLaunch();

    bool processCommand(CommandData& command);
};

#endif
