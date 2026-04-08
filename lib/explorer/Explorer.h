#ifndef EXPLORER_H
#define EXPLORER_H

#include "Navigator.h"
#include <LaserSensor.h>
#include <Ultrasonic.h>
#include <ServoMotor.h>

#include <set>

class Explorer {

    private:
        void vision();
        Pos calcCoordinates(Pos currPos, float dis, int servoAngle);
        
        ServoMotor _servo;
        LaserSensor _laser;
        Ultrasonic _ultrasonic;
        Navigator* _nav;

        std::set<Pos> toExplore;

    public:
        void explore(Navigator &nav);
        void stopExploring();

};

#endif