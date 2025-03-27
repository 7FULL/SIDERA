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

    // Check for emergency stop in all states
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
                if(controlPanel.startPlatform()){
                    currentState = STARTING_PLATFORM;
                    controlPanel.updateDisplay("Starting platform");
                }
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
                currentState = CHECKING_ROCKET;

                Serial.println("Checking rocket");

                controlPanel.updateDisplay("Checking rocket");
            }
            break;
        case CHECKING_ROCKET:
            controlPanel.receiveCommands();

            if (controlPanel.isRocketReady()) {
                currentState = ROCKET_READY;
                Serial.println("Rocket ready");

                controlPanel.updateDisplay("Rocket ready");
            }
            break;
        case ROCKET_READY:
            if (controlPanel.isCountdownInitiated()) {
                currentState = WAITING_FOR_LAUNCH;
                Serial.println("Checking for launch");

                controlPanel.updateDisplay("Checking for launch");
            }
            break;
        case WAITING_FOR_LAUNCH:
            if (controlPanel.isLaunchStopped()) {
                //TODO
            }
            if (controlPanel.countdown()) {
                if(controlPanel.launchRocket()){
                    currentState = ROCKET_FLYING;
                    Serial.println("Launching");
                    controlPanel.updateDisplay("Launching");
                }
            }
            break;
        case ROCKET_FLYING:
//            if (controlPanel.isEmergencyStop()) {
//                currentState = IDLE;
//                Serial.println("Emergency stop");
//            }
                controlPanel.receiveTelemetry();
            if (controlPanel.hasRocketLanded()){
                currentState = ROCKET_LANDED;
                Serial.println("Rocket has landed");
            }
            break;
        case ROCKET_LANDED:
//            if (controlPanel.isEmergencyStop()) {
//                currentState = IDLE;
//                Serial.println("Emergency stop");
//            }
            controlPanel.receiveTelemetry();
            break;
    }
}

State StateMachine::getState() {
    return currentState;
}