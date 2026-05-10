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
ServoMotor servo;
LaserSensor ls;
Ultrasonic ultrasonic;

void setup()
{
    Serial.begin(115200);
    // while (!Serial);

    Serial.println("Core 0 funzionante");

    initRobot();
}

void loop()
{
    explorer.update();

    robotMov.update();
}

void setup1()
{
    Serial.println("Core 1 funzionante");

    wifi.init();
    conn.init(nav, explorer);
}

void loop1()
{
    conn.update();
}

void leftTick()
{
    robotMov.getLeftEnc().increment();
}

void rightTick()
{
    robotMov.getRightEnc().increment();
}

void initRobot()
{

    explorer.init(&nav, &robotMov, &servo, &ls, &ultrasonic);

    robotMov.init(&nav);
    robotMov.getLeftEnc().init(leftTick);
    robotMov.getRightEnc().init(rightTick);

    servo.init();

    ls.init();

    ultrasonic.init();
}