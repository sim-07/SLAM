#ifndef EXPLORER_H
#define EXPLORER_H

#include "Navigator.h"
#include <LaserSensor.h>
#include <Ultrasonic.h>
#include <ServoMotor.h>
#include <RobotMovements.h>

#include <set>

static const uint8_t SCAN_PRECISION = 5; // ogni quanti gradi scansiona

enum ExpState
{
    IDLE,
    START_EXPLORING,
    SCAN,
    MOVE_TO_FRONTIER,
    COMPLETED
};

class Explorer
{

private:
    void searchObstacles();
    void scan();
    Route findBorder();
    Pos calcCoordinates(Pos currPos, float dis, int servoAngle);

    Pos _firstPos;

    Navigator *_nav;
    RobotMovements *_rb;
    ServoMotor *_servo;
    LaserSensor *_laser;
    Ultrasonic *_ultrasonic;

    ExpState _currentState = IDLE;

    std::set<Pos> _toExplore;
    volatile bool _scanning = false;
    
public:
    void init(Navigator *n, RobotMovements *r, ServoMotor *s, LaserSensor *l, Ultrasonic *u);
    void explore(Navigator &nav);
    void stopExploring();
    void update();
    void setCurrentState(ExpState currState);

    bool _isExploring;
};

#endif