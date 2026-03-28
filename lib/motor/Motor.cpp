#include "Motor.h"

void Motor::initMotor()
{
    pinMode(_pinIN1, OUTPUT);
    pinMode(_pinIN2, OUTPUT);
    pinMode(_pinPWM, OUTPUT);

    motorStop();
}

void Motor::setPower(int power)
{
    _currentPower = constrain(power, -255, 255);

    if (_currentPower > 0)
    {
        digitalWrite(_pinIN1, HIGH);
        digitalWrite(_pinIN2, LOW);
        analogWrite(_pinPWM, _currentPower);
    }
    else if (_currentPower < 0)
    {
        digitalWrite(_pinIN1, LOW);
        digitalWrite(_pinIN2, HIGH);
        analogWrite(_pinPWM, abs(_currentPower));
    }
    else
    {
        motorStop();
    }
}

void Motor::motorStop()
{
    int tempPower = abs(_currentPower);
    int smoothVal = 20;

    while (tempPower >= smoothVal)
    {
        tempPower -= smoothVal;
        if (_currentPower > 0)
        {
            digitalWrite(_pinIN1, HIGH);
            digitalWrite(_pinIN2, LOW);
            analogWrite(_pinPWM, tempPower);
        }
        else if (_currentPower < 0)
        {
            digitalWrite(_pinIN1, LOW);
            digitalWrite(_pinIN2, HIGH);
            analogWrite(_pinPWM, tempPower);
        }

        delay(30);
    }

    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, LOW);
    analogWrite(_pinPWM, 0);

    _currentPower = 0;
}