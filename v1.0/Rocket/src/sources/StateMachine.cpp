#include "headers/StateMachine.h"
#include "FS.h"

StateMachine::StateMachine() {
    currentState = IDLE;
}

void StateMachine::setup() {
    if (!rocket.setup()){
        currentState = ERROR;
    }
}

/*

TODO:
 - EMERGENCY STOP
 - VOLTAGE MONITOR

*/
const bool demo = false;

void StateMachine::update() {
    rocket.updateBuzzer();
    rocket.updateLeds();
    static unsigned long previousMillis = 0;

    switch (currentState) {
        case ERROR:
            //TODO Recibios el ok del panel de control
            if (demo){
                if (millis() - previousMillis >= 5000) {
                    previousMillis = millis();
                    currentState = IDLE;
                    rocket.resetLeds();

                    rocket.logData("[STATE] IDLE");
                }
            }else{
                if (millis() - previousMillis >= 5000) {
                    previousMillis = millis();
                    currentState = IDLE;
                    rocket.resetLeds();

                    rocket.logData("[STATE] IDLE");
                }
            }
            break;
        case IDLE:
            //TODO: DEEP SLEEP

            rocket.receiveCommands();

            if (demo){
                if (millis() - previousMillis >= 5000) {
                    previousMillis = millis();
                    currentState = WAKING_UP;

                    rocket.logData("[STATE] WAKING_UP");
                }
            }else{
                if (rocket.hasReceivedWakeUp()){
                    currentState = WAKING_UP;

                    rocket.logData("[STATE] WAKING_UP");
                }
            }
            break;
        case WAKING_UP:
            //TODO: Esperar a que el cohete se despierte

            if(demo){
                if (millis() - previousMillis >= 5000) {
                    previousMillis = millis();
                    currentState = CHECKING_ROCKET;

                    rocket.logData("[STATE] CHECKING_ROCKET");
                }
            }else{
                if (millis() - previousMillis >= 5000) {
                    previousMillis = millis();
                    currentState = CHECKING_ROCKET;

                    rocket.logData("[STATE] CHECKING_ROCKET");
                }
            }
            break;
        case CHECKING_ROCKET:
            if (rocket.checkSensors()){
                currentState = WAITING_FOR_LAUNCH;
                previousMillis = millis();

                rocket.logData("[STATE] WAITING_FOR_LAUNCH");
            } else {
                previousMillis = millis();
                currentState = ERROR;

                rocket.logData("[STATE] ERROR");
            }
            break;
        case WAITING_FOR_LAUNCH:
            rocket.receiveCommands();

            if (demo){
                if (millis() - previousMillis >= 10000) {
                    previousMillis = millis();
                    currentState = FLIGHT;

                    rocket.logData("[STATE] FLIGHT");
                }
            }else{
                if (rocket.hasReceivedLunch()){
                    currentState = FLIGHT;

                    rocket.logData("[STATE] FLIGHT");
                }
            }
            break;
        case FLIGHT:
            rocket.gravityIsOverrated();

            if (demo){
                if (rocket.isDescending()){
                    currentState = DESCENT;

                    rocket.logData("[STATE] DESCENT");
                }
//                if (millis() - previousMillis >= 10000) {
//                    previousMillis = millis();
//                    currentState = DESCENT;
//
//                    rocket.logData("[STATE] DESCENT");
//                }
            }else{
                if (rocket.isDescending()){
                    currentState = DESCENT;

                    rocket.logData("[STATE] DESCENT");
                }
            }
            break;
        case DESCENT:
            rocket.deployParachute();

            currentState = PARACHUTE_DESCENT;

            rocket.logData("[STATE] PARACHUTE_DESCENT");
            break;
        case PARACHUTE_DESCENT:
            rocket.checkParachuteDescent();

            if (rocket.hasLanded()){
                rocket.logData("[STATE] LANDED");

                currentState = LANDED;
            }
            break;
        case LANDED:
            rocket.landed();
            break;
    }
}