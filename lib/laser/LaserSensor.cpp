#include "LaserSensor.h"

LaserSensor::LaserSensor() {
    _status = false;
}

bool LaserSensor::init() {
    Wire.begin();
    sensor.setTimeout(500);
    
    if (!sensor.init()) {
        _status = false;
        return false;
    }

    sensor.setROISize(6, 6);
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(100000);
    sensor.startContinuous(120);
    
    _status = true;
    return true;
}

float LaserSensor::getDistance() {
    if (!_status) return -1.0f;

    uint16_t disMM = sensor.read();

    if (sensor.timeoutOccurred()) return -1.0f;

    return (float)disMM / 10.0f;
}

bool LaserSensor::isReady() {
    return _status;
}