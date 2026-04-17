#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "Servo.h"

class ServoMotor
{

    private:
        static const uint8_t SERVO_PIN = 15;

        Servo _internalServo;
        int _currentAngle;

    public:
        void init();
        void moveToAngle(int angle);
        void moveToAngleFast(int angle);
        int getAngle() const { return _currentAngle; }

        static const uint8_t MIN_ANGLE = 5;
        static const uint8_t MAX_ANGLE = 175;
};

#endif