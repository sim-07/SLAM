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

enum RbState {
    IDLE_RB,
    FOLLOWING,
    COMPLETED_ROUTE
};

class RobotMovements {
    private:
        Navigator *_nav;

        Motor _leftMotor;
        Motor _rightMotor;

        Encoder _leftEnc;
        Encoder _rightEnc;

        RbState _currentState = IDLE_RB;

        Route* _currentRoute = nullptr;
        int _indexRoute = 0;

        const float _wheelDistance = 10.0;

    public:
        RobotMovements()
            : _leftMotor(6, 7, 8),
              _rightMotor(9, 10, 11),
              _leftEnc(20),
              _rightEnc(21)
        {}

        void init(Navigator *n);
        void goStraight(float dis, float power = 150);
        void turn(float angle, float power = 150);
        void stop();
        void setCurrentState(RbState currState);
        void update();
        void setRoute(Route &route);

        Encoder& getLeftEnc() { return _leftEnc; }
        Encoder& getRightEnc() { return _rightEnc; }

        NavStatus followPath();

};

#endif


