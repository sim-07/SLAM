#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include <Wire.h>
#include <VL53L1X.h>

class LaserSensor
{
private:
    VL53L1X sensor;
    bool _status;

public:
    LaserSensor();

    bool init();
    float getDistance();
    bool isReady();
};

#endif