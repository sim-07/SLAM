#include "Ultrasonic.h"

void Ultrasonic::initUltrasonic()
{
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    digitalWrite(_trigPin, LOW);
}

float Ultrasonic::getDistance()
{
    int v_scan = 0;
    float sum_d = 0;
    float distance = 0;

    for (int i = 0; i < 3; i++)
    {
        digitalWrite(_trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(_trigPin, LOW);

        float tempDur = pulseIn(_echoPin, HIGH, 30000);
        float tempDis = tempDur * 0.0343f / 2;

        if (tempDis > _minDistance && tempDis < _maxDistance)
        {
            v_scan++;
            sum_d += tempDis;
        }

        delay(5);
    }

    if (v_scan == 0)
    {
        Serial.println("Error Out of Range");
        return -1.0;
    }

    distance = sum_d / v_scan;

    return distance;
}