#include "Ultrasonic.h"

void Ultrasonic::init()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);
}

float Ultrasonic::getDistance()
{
    int v_scan = 0;
    float sum_d = 0;
    float distance = 0;

    for (int i = 0; i < 3; i++)
    {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);

        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        float tempDur = pulseIn(ECHO_PIN, HIGH, 30000);
        float tempDis = tempDur * 0.0343f / 2;

        if (tempDis > MIN_DISTANCE && tempDis < MAX_DISTANCE)
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