#include "RobotMovements.h"

void RobotMovements::init()
{

    _leftMotor.initMotor();
    _rightMotor.initMotor();

    stop();
}

const float Kp = 10; // da calibrare

void RobotMovements::straight(float dis, float power)
{

    _leftEnc.reset();
    _rightEnc.reset();

    float avg = 0;
    while (avg < abs(dis)) {
        float dL = abs(_leftEnc.getCurrDistance());
        float dR = abs(_rightEnc.getCurrDistance());

        avg = (dL + dR) / 2.0;

        float error = dL - dR;

        if (abs(error) > 1.1) {
            float corr = error * Kp;

            _leftMotor.setPower(power - corr);
            _rightMotor.setPower(power + corr);
        } else {
            _leftMotor.setPower(power);
            _rightMotor.setPower(power);
        }
        
        delay(10);
    }

    stop();
}

void RobotMovements::stop()
{
    _leftMotor.motorStop();
    _rightMotor.motorStop();
}

void RobotMovements::turn(float angle, float power) {
    _leftEnc.reset();
    _rightEnc.reset();

    float turnDis = abs((_wheelDistance * PI * angle) / 360); // somma distanza che le ruote devono percorrere

    float dir = (angle > 0) ? 1.0 : -1.0;

    float avg = 0;

    while (avg < turnDis) {
        float abs_dis_l = abs(_leftEnc.getCurrDistance());
        float abs_dis_r = abs(_rightEnc.getCurrDistance());

        avg = (abs_dis_l + abs_dis_r) / 2;

        float error = abs_dis_l - abs_dis_r;

        if (abs(error) > 0.2) {
            float corr = error * Kp;

            _leftMotor.setPower((power - corr) * dir);
            _rightMotor.setPower((power + corr) * -dir);
        } else {
            _leftMotor.setPower(power * dir);
            _rightMotor.setPower(power * -dir);
            
        }
        
        delay(10);
    }

    stop();

}

NavStatus followPath(Route &route) {
    // TODO
}