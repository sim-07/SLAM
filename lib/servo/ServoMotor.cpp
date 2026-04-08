#include "ServoMotor.h"

void ServoMotor::init() {
    _internalServo.attach(SERVO_PIN);
    
    moveToAngle(5);
}

void ServoMotor::moveToAngle(int angle) {
    angle = constrain(angle, 5, 175);

    int step = (angle > _currentAngle) ? 1 : -1;

    while (_currentAngle != angle) {
        _currentAngle += step;
        _internalServo.write(_currentAngle);

        delay(10);
    }
    
}