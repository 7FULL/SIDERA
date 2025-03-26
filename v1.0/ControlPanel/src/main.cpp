#include <Arduino.h>
#include "headers/StateMachine.h"

StateMachine stateMachine;

void setup() {
    Serial.begin(115200);

    stateMachine.setup();
}

void loop() {
    stateMachine.update();
}