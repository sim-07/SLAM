#include "LaserSensor.h"

LaserSensor::LaserSensor() {
    _status = false;
    _lastDistance = 0;
}

bool LaserSensor::init() {
    Wire.begin();
    sensor.setTimeout(500);
    
    if (!sensor.init()) {
        _status = false;
        return false;
    }

    
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
    
    _status = true;
    return true;
}

int LaserSensor::getDistance() {
    if (!_status) return -1;
    
    _lastDistance = sensor.read();
    return _lastDistance;
}

bool LaserSensor::isReady() {
    return _status;
}