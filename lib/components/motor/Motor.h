#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {

    private:
        const int _pinIN1;
        const int _pinIN2;
        const int _pinPWM;
        int _currentPower;
        

    public:
        Motor(int in1, int in2, int pwm) 
            : _pinIN1(in1), _pinIN2(in2), _pinPWM(pwm), _currentPower(0) {}

        void initMotor();
        void setPower(int power);
        void motorStop();
        int getPower() { return _currentPower; }
};

#endif