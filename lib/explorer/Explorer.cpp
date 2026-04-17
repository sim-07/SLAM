#include "Explorer.h"
#include "Navigator.h"

void Explorer::explore(Navigator &nav)
{
    _nav = &nav;
    _servo.init();
    _ultrasonic.init();
    _laser.init();

    // TODO
    scan();
    _rb.turn(180);
    scan();

    findBorder();

}

Pos Explorer::calcCoordinates(Pos currPos, float dis, int servoAngle)
{
    float rad = (servoAngle * PI) / 180.0f;
    float nX = cos(rad) * dis + (float)currPos.x;
    float nY = sin(rad) * dis + (float)currPos.y;

    return {(int16_t)round(nX), (int16_t)round(nY)};
}

void Explorer::scan()
{

    if (_servo.getAngle() != ServoMotor::MIN_ANGLE)
    {
        _servo.moveToAngleFast(ServoMotor::MIN_ANGLE);
        delay(200);
    }

    for (int a = ServoMotor::MIN_ANGLE; a <= ServoMotor::MAX_ANGLE; a += SCAN_PRECISION)
    {
        _servo.moveToAngle(a);
        searchObstacles();
        delay(30);
    }
}

void Explorer::searchObstacles()
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
        _nav->sculpt(c.x, c.y, Navigator::LASER);
    }

    ultrasonicDis = _ultrasonic.getDistance();
    Pos c = calcCoordinates(p, ultrasonicDis, totAngle);
    _nav->sculpt(c.x, c.y, Navigator::ULTRASONIC);
}


void Explorer::findBorder()
{

}