#include <Arduino.h>

#include "RobotMovements.h"

void leftTick();
void rightTick();
void initRobot();

RobotMovements robotMov;



void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    initRobot();
}

void loop() {

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