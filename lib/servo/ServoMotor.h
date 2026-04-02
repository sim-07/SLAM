#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "Servo.h"

class ServoMotor {

    private:
        const int _servoPin;
        Servo _internalServo;

    public:
        ServoMotor(int servoPin) : _servoPin(servoPin) {}
    
        void initServo();
        void moveToAngle(int angle);
};

#endif