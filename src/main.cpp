#include <Arduino.h>

#include "RobotMovements.h"
#include "Explorer.h"
#include "Navigator.h"

void leftTick();
void rightTick();
void initRobot();

RobotMovements robotMov;
Navigator nav;
Explorer exp;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    initRobot();
    
}

void loop() {

}

void setup1() {
    // TODO wifi
}

void loop1() {
    // TODO wifi
}


void leftTick() {
    robotMov.getLeftEnc().increment();
}

void rightTick() {
    robotMov.getRightEnc().increment();
}

void initRobot() {
    
    robotMov.init(); 
    robotMov.getLeftEnc().init(leftTick);
    robotMov.getRightEnc().init(rightTick);
}