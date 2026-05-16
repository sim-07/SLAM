#include "RobotMovements.h"

void RobotMovements::init(Navigator *n)
{

    _leftMotor.initMotor();
    _rightMotor.initMotor();

    _nav = n;

    stop();
}

const float Kp = 10; // da calibrare
const int UNIT = 10; // dimensioni cella in cm

void RobotMovements::update()
{
    switch (_currentState)
    {
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

void RobotMovements::setRoute(Route &route)
{
    _currentRoute = &route;
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

        _targetDis = 0;
        _avgStraight = 0;
        _nav->setCurrPos(_currentRoute->route[_currIndexRoute].x, _currentRoute->route[_currIndexRoute].y);
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
        _targetAngle = 0;
        _avgTurn = 0;
        _currentState = FOLLOWING;
    }
}

void RobotMovements::followPath()
{

    if (_currentRoute->route.size() > _currIndexRoute)
    {
        _targetCell = _currentRoute->route[_currIndexRoute];
    } else {
        _currentState = COMPLETED_ROUTE;
        return;
    }
    
    Pos currPos = _nav->getPos();
    float currDir = _nav->getDir();


    if (_targetCell.x == currPos.x && _targetCell.y == currPos.y)
    {
        _currIndexRoute++;
        return;
    }

    float absAngle = atan2(_targetCell.y - currPos.y, _targetCell.x - currPos.x) * 180.0 / PI;
    float turnAngle = absAngle - currDir;

    if (turnAngle > 180)
        turnAngle -= 360;
    if (turnAngle < -180)
        turnAngle += 360;

    if (abs(turnAngle) > 0.2)
    {
        stop();
        _leftEnc.reset();
        _rightEnc.reset();
        _avgTurn = 0;
        _targetAngle = turnAngle;
        _currentState = TURNING;
        _nav->setDir(absAngle);
        return;
    }

    bool isDiagonal = (_currentRoute->route[_currIndexRoute + 1].x != _currentRoute->route[_currIndexRoute].x && _currentRoute->route[_currIndexRoute + 1].y != _currentRoute->route[_currIndexRoute].y);
    float straightDis = isDiagonal ? 1.4142135f : 1.0f; 
    
    _currIndexRoute++;

    for (int i = _currIndexRoute; i < _currentRoute->route.size() - 1; i++)
    {
        float absAngleS = atan2(_currentRoute->route[i + 1].y - _currentRoute->route[i].y, _currentRoute->route[i + 1].x - _currentRoute->route[i].x) * 180.0 / PI;
        float turnAngleS = absAngleS - absAngle; // quanto cambia in base all'inizio (se è dritto l'angolo rimane lo stesso per tutto il tragitto)

        if (turnAngleS > 180)
            turnAngleS -= 360;
        if (turnAngleS < -180)
            turnAngleS += 360;

        if (abs(turnAngleS) > 0.2)
        {
            break;
        }

        isDiagonal = (_currentRoute->route[i + 1].x != _currentRoute->route[i].x && _currentRoute->route[i + 1].y != _currentRoute->route[i].y);
        straightDis += isDiagonal ? 1.4142135f : 1.0f;
        
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

// NavStatus RobotMovements::followPath(Route &route, Navigator &nav) {

//     while (!route.route.empty())
//     {
//         Pos target = route.route.top();
//         Pos currPos = nav.getPos();
//         float currDir = nav.getDir();

//         float absAngle = atan2(target.y - currPos.y, target.x - currPos.x) * 180.0 / PI;
//         float turnAngle = absAngle - currDir;

//         if (turnAngle > 180) turnAngle -= 360;
//         if (turnAngle < -180) turnAngle += 360;

//         if (abs(turnAngle) > 1.0) {
//             stop(); // TODO girare gradualmente
//             turn(turnAngle);
//             nav.setDir(absAngle);
//         }

//         // goStraight finché il percorso è dritto
//         Pos currPointS = currPos;
//         float currDirS = currDir;
//         float straightStepsCells = 0;
//         bool isDiagonal = false;
//         while (!route.route.empty()) {

//             Pos nextPointS = route.route.top();

//             float absAngleS = atan2(nextPointS.y - currPointS.y, nextPointS.x - currPointS.x) * 180.0 / PI;
//             float turnAngleS = absAngleS - absAngle; // quanto cambia in base all'inizio (se è dritto l'angolo rimane lo stesso per tutto il tragitto)

//             if (turnAngleS > 180) turnAngleS -= 360;
//             if (turnAngleS < -180) turnAngleS += 360;

//             if (abs(turnAngleS) > 1.0) {
//                 break;
//             }

//             if (nextPointS.x != currPointS.x && nextPointS.y != currPointS.y) {
//                 isDiagonal = true;
//             } else {
//                 isDiagonal = false;
//             }

//             straightStepsCells += isDiagonal ? 1.41f : 1;

//             currPointS = nextPointS;
//             route.route.pop();
//         }

//         goStraight(straightStepsCells * UNIT);
//         nav.setCurrPos(currPointS.x, currPointS.y);

//     }

//     // TODO gestire rilevamento ostacoli
//     return SUCCESS;

// }