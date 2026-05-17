#ifndef ROBOTMOVEMENTS_H
#define ROBOTMOVEMENTS_H

#include <Arduino.h>

#include "Encoder.h"
#include "Navigator.h"
#include "Motor.h"

static const uint8_t MOTOR_POWER = 150;

// enum NavStatus {
//     SUCCESS = 0,
//     OBSTACLE = 1,
//     FAILED = 2
// };

enum RbState {
    IDLE_RB,
    MOVING_STRAIGHT,
    TURNING,
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

        int _currIndexRoute = 0;

        Pos _targetCell;

        float _avgStraight = 0;
        float _avgTurn = 0;
        float _targetDis = 0;
        float _targetAngle = 0;

        float normAngle(float angle);

        const float _wheelDistance = 10.0;

    public:
        RobotMovements()
            : _leftMotor(6, 7, 8),
              _rightMotor(9, 10, 11),
              _leftEnc(20),
              _rightEnc(21)
        {}

        void init(Navigator *n);
        void goStraight();
        void turn();
        void stop();
        void setCurrentState(RbState currState);
        RbState getCurrentState() { return _currentState; };
        void update();
        void setRoute(Route &route);
        void followPath();

        Encoder& getLeftEnc() { return _leftEnc; }
        Encoder& getRightEnc() { return _rightEnc; }

};

#endif


