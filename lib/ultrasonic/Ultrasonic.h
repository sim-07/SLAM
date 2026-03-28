#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {

    private:
        const int _echoPin;
        const int _trigPin;
        const int _maxDistance;
        const int _minDistance;

    public:
        Ultrasonic(int echo, int trig, int maxDistance, int minDistance) : _echoPin(echo), _trigPin(trig), _maxDistance(maxDistance), _minDistance(minDistance) {}
    
        void initUltrasonic();
        float getDistance();
};

#endif