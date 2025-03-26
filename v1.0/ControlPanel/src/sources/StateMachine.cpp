#include "headers/StateMachine.h"
#include <Arduino.h>

StateMachine::StateMachine() {
    currentState = IDLE;
}

void StateMachine::setup() {
    controlPanel.begin();
}

void StateMachine::update() {
    controlPanel.updateBuzzer();

    if (controlPanel.isEmergencyStop()) {
        controlPanel.emergencyStop();
        currentState = IDLE;
        controlPanel.updateDisplay("EMERGENCY STOP");
        Serial.println("Emergency stop activated");
        delay(100);
        return;
    }
    switch (currentState) {
        case IDLE:
            if (controlPanel.isLaunchPlatformInitiated()) {
                currentState = STARTING_PLATFORM;
                Serial.println("Starting platform");

                controlPanel.startPlatform();

                controlPanel.updateDisplay("Starting platform");
            }
            break;
        case STARTING_PLATFORM:
            //TODO: Wait for the platform to start
            currentState = PLATFORM_STARTED;

            Serial.println("Platform started");

            controlPanel.updateDisplay("Platform started");
            break;
        case PLATFORM_STARTED:
            if (controlPanel.isWakeupInitiated()) {
                currentState = WAKING_UP_ROCKET;
                Serial.println("Waking up rocket");

                controlPanel.updateDisplay("Waking up rocket");
            }
            break;
        case WAKING_UP_ROCKET:
            if (controlPanel.wakeUp()){
                //TODO: Send message to wake up the rocket
                currentState = CHECKING_ROCKET;

                Serial.println("Checking rocket");

                controlPanel.updateDisplay("Checking rocket");
            }
            break;
        case CHECKING_ROCKET:
            //TODO: Check if the rocket is ready
            currentState = ROCKET_READY;
            Serial.println("Rocket ready");

            controlPanel.updateDisplay("Rocket ready");
            break;
        case ROCKET_READY:
            if (controlPanel.isCountdownInitiated()) {
                currentState = CHECKING_FOR_LAUNCH;
                Serial.println("Checking for launch");

                controlPanel.updateDisplay("Checking for launch");
            }
            break;
        case CHECKING_FOR_LAUNCH:
            //TODO: Check if the rocket is ready to launch
            currentState = WAITING_FOR_LAUNCH;
            Serial.println("Waiting for launch");
            break;
        case WAITING_FOR_LAUNCH:
            if (controlPanel.countdown()) {
                currentState = LAUNCHING;
                Serial.println("Launching");
                controlPanel.updateDisplay("Launching");
            }
            break;
        case LAUNCHING:
            if (controlPanel.isLaunchStopped()) {
                currentState = ROCKET_FLYING;
                Serial.println("Launch stopped");
            }
            break;
        case ROCKET_FLYING:
            if (controlPanel.isEmergencyStop()) {
                currentState = IDLE;
                Serial.println("Emergency stop");
            }
            break;
        case ROCKET_LANDED:
            if (controlPanel.isEmergencyStop()) {
                currentState = IDLE;
                Serial.println("Emergency stop");
            }
            break;
    }
}

State StateMachine::getState() {
    return currentState;
}