#include "RobotMovements.h"

void RobotMovements::init(Navigator *n, QueueHandle_t messToClient)
{
    _leftMotor.initMotor();
    _rightMotor.initMotor();

    _messToClient = messToClient;

    _nav = n;

    stop();
}

const float Kp = 10; // da calibrare
const int UNIT = 10; // dimensioni cella in cm

void RobotMovements::update()
{
    switch (_currentState)
    {
    case GOTO:
        goTo();
        break;

    case FOLLOWING:
        followPath();
        break;

    case MOVING_STRAIGHT:
        goStraight();
        break;

    case TURNING:
        turn();
        break;

    case COMPLETED_ROUTE:
        stop();
        _currIndexRoute = 0;
        break;

    default:
        break;
    }
}

void RobotMovements::goTo()
{
    Route r = _nav->calcRoute(_destination.x, _destination.y);

    if (r.numSteps > 0)
    {
        _currentState = FOLLOWING;
        _currentRoute = r;
    }
    else
    {
        Message msg;
        msg.type = INFO;

        strncpy(msg.mess, "No suitable path", sizeof(msg.mess) - 1);
        msg.mess[sizeof(msg.mess) - 1] = '\0';

        xQueueSend(_messToClient, &msg, 0);
    }
}

void RobotMovements::setDestination(int16_t x, int16_t y)
{
    _destination = {x, y};
}

void RobotMovements::setRoute(Route &route)
{
    _currentRoute = route;
    _currIndexRoute = 0;
}

void RobotMovements::setCurrentState(RbState currState)
{
    _currentState = currState;
}

void RobotMovements::goStraight()
{

    if (_avgStraight < abs(_targetDis))
    {
        float dL = abs(_leftEnc.getCurrDistance());
        float dR = abs(_rightEnc.getCurrDistance());

        _avgStraight = (dL + dR) / 2.0;

        float error = dL - dR;

        if (abs(error) > 1.1)
        {
            float corr = error * Kp;

            _leftMotor.setPower(MOTOR_POWER - corr);
            _rightMotor.setPower(MOTOR_POWER + corr);
        }
        else
        {
            _leftMotor.setPower(MOTOR_POWER);
            _rightMotor.setPower(MOTOR_POWER);
        }
    }
    else
    {
        stop();
        _leftEnc.reset();
        _rightEnc.reset();
        _targetDis = 0;
        _avgStraight = 0;
        _nav->setCurrPos(_currentRoute.route[_currIndexRoute].x, _currentRoute.route[_currIndexRoute].y); // TODO controllare
        _currentState = FOLLOWING;
    }
}

void RobotMovements::stop()
{
    _leftMotor.motorStop();
    _rightMotor.motorStop();
}

void RobotMovements::turn()
{
    // angle > 0 --> destra
    // TODO giroscopio

    float turnDis = abs((_wheelDistance * PI * _targetAngle) / 360); // totale distanza che le ruote devono percorrere
    float dir = (_targetAngle > 0) ? 1.0 : -1.0;

    if (_avgTurn < turnDis)
    {
        float abs_dis_l = abs(_leftEnc.getCurrDistance());
        float abs_dis_r = abs(_rightEnc.getCurrDistance());

        _avgTurn = (abs_dis_l + abs_dis_r) / 2;

        float error = abs_dis_l - abs_dis_r;

        if (abs(error) > 0.2)
        {
            float corr = error * Kp;

            _leftMotor.setPower((MOTOR_POWER - corr) * dir);
            _rightMotor.setPower((MOTOR_POWER + corr) * -dir);
        }
        else
        {
            _leftMotor.setPower(MOTOR_POWER * dir);
            _rightMotor.setPower(MOTOR_POWER * -dir);
        }
    }
    else
    {
        stop();
        _leftEnc.reset();
        _rightEnc.reset();
        _nav->setDir(normAngle(_nav->getDir() + _targetAngle));
        _targetAngle = 0;
        _avgTurn = 0;
        _currentState = FOLLOWING;
    }
}

void RobotMovements::followPath()
{

    if (_currentRoute.route.size() > _currIndexRoute)
    {
        _nextCell = _currentRoute.route[_currIndexRoute];
    }
    else
    {
        _currentState = COMPLETED_ROUTE;
        return;
    }

    Pos currPos = _nav->getPos();
    float currDir = _nav->getDir();

    if (_nextCell.x == currPos.x && _nextCell.y == currPos.y)
    {
        _currIndexRoute++;
        return;
    }

    float absAngle = atan2(_nextCell.y - currPos.y, _nextCell.x - currPos.x) * 180.0 / PI;
    float turnAngle = absAngle - currDir;

    turnAngle = normAngle(turnAngle);

    if (abs(turnAngle) > 0.2)
    {
        stop();
        _leftEnc.reset();
        _rightEnc.reset();
        _avgTurn = 0;
        _targetAngle = turnAngle;
        _currentState = TURNING;

        return;
    }

    // va dritto finché non c'è una curva

    bool isDiagonal = false;
    float straightDis = 0;

    for (int i = _currIndexRoute; i < _currentRoute.route.size() - 1; i++)
    {
        float absAngleS = atan2(_currentRoute.route[i + 1].y - _currentRoute.route[i].y, _currentRoute.route[i + 1].x - _currentRoute.route[i].x) * 180.0 / PI;
        float turnAngleS = absAngleS - absAngle; // quanto cambia in base all'inizio (se è dritto l'angolo rimane lo stesso per tutto il tragitto)

        turnAngleS = normAngle(turnAngleS);

        isDiagonal = (_currentRoute.route[i + 1].x != _currentRoute.route[i].x && _currentRoute.route[i + 1].y != _currentRoute.route[i].y);
        straightDis += isDiagonal ? 1.4142135f : 1.0f;

        if (abs(turnAngleS) > 0.1)
        {
            break;
        }

        _currIndexRoute++;
    }

    _leftEnc.reset();
    _rightEnc.reset();
    _avgStraight = 0;
    _targetDis = straightDis * UNIT;
    _currentState = MOVING_STRAIGHT;

    // TODO gestire rilevamento ostacoli
    return;
}

float RobotMovements::normAngle(float angle)
{
    if (angle > 180)
        angle -= 360;
    if (angle < -180)
        angle += 360;

    return angle;
}