#include "ServoMotor.h"

void ServoMotor::init() {
    _internalServo.attach(SERVO_PIN);
    
    moveToAngle(5);
}

void ServoMotor::moveToAngle(int angle) {
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);

    int step = (angle > _currentAngle) ? 1 : -1;

    while (_currentAngle != angle) {
        _currentAngle += step;
        _internalServo.write(_currentAngle);

        delay(10);
    }
    
}

void ServoMotor::moveToAngleFast(int angle) {
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
    _internalServo.write(_currentAngle);
}