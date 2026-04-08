#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "Servo.h"

static const uint8_t SERVO_PIN = 15;

class ServoMotor {

    private:
        Servo _internalServo;
        int _currentAngle;

    public:    
        void init();
        void moveToAngle(int angle);
        int getAngle() const { return _currentAngle; }
};

#endif