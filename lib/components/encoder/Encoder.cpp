#include "Encoder.h"

bool Encoder::init(void (*ISR_callback)(void)) {
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), ISR_callback, RISING);

    if (digitalRead(_pin) == LOW) {
        return false;
    }

    return true;
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

float Encoder::getCurrDistance() { // cm
    float c = _wheelDiameter * PI;
    return ((float)_pulseCount / _spaces) * c;
}