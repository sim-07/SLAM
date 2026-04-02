#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {

    private:
        const int _pin;
        const float _spaces;
        const float _wheelDiameter;
        volatile long _pulseCount;


    public:
        Encoder(int pin, float spaces = 20.0, float wheelDiameter = 6.5)
            : _pin(pin), _pulseCount(0), _spaces(spaces), _wheelDiameter(wheelDiameter) {}

        void init(void (*ISR_callback)(void));
        void increment();
        void reset();
        
        long getPulses();
        float getCurrDistance();
};

#endif