#ifndef EXPLORER_H
#define EXPLORER_H

#include "Navigator.h"
#include <LaserSensor.h>
#include <Ultrasonic.h>
#include <ServoMotor.h>
#include <RobotMovements.h>

#include <set>

static const uint8_t SCAN_PRECISION = 5; // ogni quanti gradi scansiona

class Explorer {

    private:
        void searchObstacles();
        void scan();
        void findBorder();
        Pos calcCoordinates(Pos currPos, float dis, int servoAngle);
        
        ServoMotor _servo;
        LaserSensor _laser;
        Ultrasonic _ultrasonic;
        Navigator* _nav;
        RobotMovements _rb;

        std::set<Pos> _toExplore;
        volatile bool _scanning = false;

    public:
        void explore(Navigator &nav);
        void stopExploring();

};

#endif