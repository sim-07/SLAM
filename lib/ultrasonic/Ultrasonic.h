#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

static const uint8_t TRIG_PIN = 17;
static const uint8_t ECHO_PIN = 16;
static const uint8_t MIN_DISTANCE = 2;
static const uint8_t MAX_DISTANCE = 400;

class Ultrasonic
{

    public:
        void init();
        float getDistance();
};

#endif