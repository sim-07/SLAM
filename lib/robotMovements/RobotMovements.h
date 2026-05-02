#ifndef ROBOTMOVEMENTS_H
#define ROBOTMOVEMENTS_H

#include <Arduino.h>

#include "Encoder.h"
#include "Navigator.h"
#include "Motor.h"


enum NavStatus {
    SUCCESS = 0,
    OBSTACLE = 1,
    FAILED = 2
};

class RobotMovements {
    private:
        Motor _leftMotor;
        Motor _rightMotor;

        Encoder _leftEnc;
        Encoder _rightEnc;

        const float _wheelDistance = 10.0;

    public:
        RobotMovements()
            : _leftMotor(6, 7, 8),
              _rightMotor(9, 10, 11),
              _leftEnc(20),
              _rightEnc(21)
        {}

        void init();
        void goStraight(float dis, float power = 150);
        void turn(float angle, float power = 150);
        void stop();

        Encoder& getLeftEnc() { return _leftEnc; }
        Encoder& getRightEnc() { return _rightEnc; }

        NavStatus followPath(Route &route, Navigator &nav);

};

#endif


