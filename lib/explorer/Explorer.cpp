#include "Explorer.h"
#include "Navigator.h"

void Explorer::explore(Navigator &nav)
{
    _nav = &nav;
    _servo.init();
    _ultrasonic.init();
    _laser.init();

    // TODO mettere in toExplore il punto centrale tra due chunks e trovare percorso con A*

    vision();
}

void Explorer::vision()
{
    float laserDis = -1;
    float ultrasonicDis = -1;
    int servoAngle = _servo.getAngle();
    int robotAngle = _nav->getDir();
    int totAngle = servoAngle + robotAngle;

    Pos p = _nav->getPos();

    if (_laser.isReady())
    {
        laserDis = _laser.getDistance();
        Pos c = calcCoordinates(p, laserDis, totAngle);
        _nav->addObstacle(c.x, c.y, Navigator::LASER);
    }

    ultrasonicDis = _ultrasonic.getDistance();
    Pos c = calcCoordinates(p, ultrasonicDis, totAngle);
    _nav->addObstacle(c.x, c.y, Navigator::ULTRASONIC);
}

Pos Explorer::calcCoordinates(Pos currPos, float dis, int servoAngle)
{
    float rad = (servoAngle * PI) / 180.0f;
    float nX = cos(rad) * dis + (float)currPos.x;
    float nY = sin(rad) * dis + (float)currPos.y;

    return { (int16_t)round(nX), (int16_t)round(nY) };
}