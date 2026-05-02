#include <Arduino.h>

#include "RobotMovements.h"
#include "Explorer.h"
#include "Navigator.h"
#include <WifiConn.h>
#include <Connection.h>

void leftTick();
void rightTick();
void initRobot();

RobotMovements robotMov;
Navigator nav;
Explorer explorer;
WifiConn wifi;
Connection conn;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    initRobot();
    
}

void loop() {

}

void setup1() {
    wifi.init();
    conn.init(nav, explorer);
}

void loop1() {
    conn.update();
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