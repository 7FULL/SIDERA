#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Rocket.h"
#include <Arduino.h>

enum States {
    IDLE,
    WAKING_UP,
    CHECKING_ROCKET,
    WAITING_FOR_LAUNCH,
    FLIGHT,
    DESCENT,
    PARACHUTE_DESCENT,
    LANDED,
    ERROR
};

class StateMachine {
public:
    StateMachine();
    void update();
    void setup();

private:
    States currentState;

    Rocket rocket;
};

#endif