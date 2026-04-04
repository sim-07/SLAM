#include "RobotMovements.h"

void RobotMovements::init()
{

    _leftMotor.initMotor();
    _rightMotor.initMotor();

    stop();
}

const float Kp = 10; // da calibrare
const int UNIT = 15; // dimensioni cella in cm

void RobotMovements::goStraight(float dis, float power)
{

    _leftEnc.reset();
    _rightEnc.reset();

    float avg = 0;
    while (avg < abs(dis)) {
        float dL = abs(_leftEnc.getCurrDistance());
        float dR = abs(_rightEnc.getCurrDistance());

        avg = (dL + dR) / 2.0;

        float error = dL - dR;

        if (abs(error) > 1.1) {
            float corr = error * Kp;

            _leftMotor.setPower(power - corr);
            _rightMotor.setPower(power + corr);
        } else {
            _leftMotor.setPower(power);
            _rightMotor.setPower(power);
        }
        
        delay(10);
    }

    stop();
}

void RobotMovements::stop()
{
    _leftMotor.motorStop();
    _rightMotor.motorStop();
}

void RobotMovements::turn(float angle, float power) { // angle > 0 --> right

    // TODO giroscopio

    _leftEnc.reset();
    _rightEnc.reset();

    float turnDis = abs((_wheelDistance * PI * angle) / 360); // totale distanza che le ruote devono percorrere
    float dir = (angle > 0) ? 1.0 : -1.0;
    float avg = 0;

    while (avg < turnDis) {
        float abs_dis_l = abs(_leftEnc.getCurrDistance());
        float abs_dis_r = abs(_rightEnc.getCurrDistance());

        avg = (abs_dis_l + abs_dis_r) / 2;

        float error = abs_dis_l - abs_dis_r;

        if (abs(error) > 0.2) {
            float corr = error * Kp;

            _leftMotor.setPower((power - corr) * dir);
            _rightMotor.setPower((power + corr) * -dir);
        } else {
            _leftMotor.setPower(power * dir);
            _rightMotor.setPower(power * -dir);
            
        }
        
        delay(10);
    }

    stop();

}

NavStatus RobotMovements::followPath(Route &route, Navigator &nav) {

    while (!route.route.empty())
    {
        Pos target = route.route.top();
        Pos currPos = nav.getPos();
        float currDir = nav.getDir();

        float absAngle = atan2(target.y - currPos.y, target.x - currPos.x) * 180.0 / PI;
        float turnAngle = absAngle - currDir;

        if (turnAngle > 180) turnAngle -= 360;
        if (turnAngle < -180) turnAngle += 360;

        if (abs(turnAngle) > 1.0) {
            stop(); // TODO girare gradualmente
            turn(turnAngle);
            nav.setDir(absAngle);
        }

        // goStraight finché il percorso è dritto
        Pos currPointS = currPos;
        float currDirS = currDir;
        float straightStepsCells = 0;
        bool isDiagonal = false;
        while (!route.route.empty()) {

            Pos nextPointS = route.route.top();

            float absAngleS = atan2(nextPointS.y - currPointS.y, nextPointS.x - currPointS.x) * 180.0 / PI;
            float turnAngleS = absAngleS - absAngle; // quanto cambia in base all'inizio (se è dritto l'angolo rimane lo stesso per tutto il tragitto)

            if (turnAngleS > 180) turnAngleS -= 360;
            if (turnAngleS < -180) turnAngleS += 360;

            if (abs(turnAngleS) > 1.0) {
                break;
            }

            if (nextPointS.x != currPointS.x && nextPointS.y != currPointS.y) {
                isDiagonal = true;
            } else {
                isDiagonal = false;
            }

            straightStepsCells += isDiagonal ? 1.41f : 1;

            currPointS = nextPointS;
            route.route.pop();
        }
        

        goStraight(straightStepsCells * UNIT);
        nav.setCurrPos(currPointS.x, currPointS.y);

    }

    // TODO gestire rilevamento ostacoli
    return SUCCESS;
    
}