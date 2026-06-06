#ifndef ROBOTMOVEMENTS_H
#define ROBOTMOVEMENTS_H

#include <Arduino.h>

#include "Encoder.h"
#include "Navigator.h"
#include "Motor.h"

#include "../../../src/Common.h"

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
    COMPLETED_ROUTE,
    GOTO
};

class RobotMovements {
    private:
        Navigator *_nav;

        Motor _leftMotor;
        Motor _rightMotor;

        Encoder _leftEnc;
        Encoder _rightEnc;

        RbState _currentState = IDLE_RB;

        Route _currentRoute;

        int _currIndexRoute = 0;

        Pos _nextCell;
        Pos _destination;

        float _avgStraight = 0;
        float _avgTurn = 0;
        float _targetDis = 0;
        float _targetAngle = 0;

        float normAngle(float angle);

        QueueHandle_t _messToClient;

        void goTo();

        const float _wheelDistance = 10.0;

    public:
        RobotMovements()
            : _leftMotor(7, 18, 6),
              _rightMotor(20, 21, 19),
              _leftEnc(4),
              _rightEnc(5)
        {}

        void init(Navigator *n, QueueHandle_t messToClient);
        void goStraight();
        void turn();
        void stop();
        void setCurrentState(RbState currState);
        RbState getCurrentState() { return _currentState; };
        void update();
        void setRoute(Route &route);
        void followPath();
        void setDestination(int16_t x, int16_t y);

        Encoder& getLeftEnc() { return _leftEnc; }
        Encoder& getRightEnc() { return _rightEnc; }

};

#endif


