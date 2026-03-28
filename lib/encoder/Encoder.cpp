#include "Encoder.h"

void Encoder::init(void (*ISR_callback)(void)) {
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), ISR_callback, RISING);
}

void Encoder::increment() {
    _pulseCount++;
}

void Encoder::reset() {
    _pulseCount = 0;
}

long Encoder::getPulses() {
    return _pulseCount;
}

float Encoder::getDistance() {
    float c = _wheelDiameter * PI;
    return ((float)_pulseCount / _spaces) * c;
}