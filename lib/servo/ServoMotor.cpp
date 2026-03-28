#include "ServoMotor.h"

void ServoMotor::initServo() {
    _internalServo.attach(_servoPin);
    
    _internalServo.write(90);
}

void ServoMotor::moveToAngle(int angle) {
    angle = constrain(angle, 0, 180);

    _internalServo.write(angle);
}