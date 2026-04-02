#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include <Wire.h>
#include <VL53L1X.h>

class LaserSensor
{
private:
    VL53L1X sensor;
    int _lastDistance;
    bool _status;

public:
    LaserSensor();

    bool init();
    int getDistance();
    bool isReady();
};

#endif