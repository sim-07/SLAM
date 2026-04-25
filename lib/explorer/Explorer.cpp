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
    float disX = cos(rad) * dis + (float)currPos.x;
    float disY = sin(rad) * dis + (float)currPos.y;

    int16_t nX = disX / Navigator::CELL_CM;
    int16_t nY = disY / Navigator::CELL_CM;

    return {nX, nY};
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
    int16_t currX = _nav->getPos().x;
    int16_t currY = _nav->getPos().y;

    std::deque<Pos> mapOpenList;
    std::set<Pos> mapClosedList;

    const auto& map = _nav->getMap();

    // TODO
    // Parte dalle coordinate del robot correnti e si espande in tutte le 8 celle adiacenti. Quindi con un for controllo singolarmente ogni cella adiacente alla cella correntemente analizzata. Se è un ostacolo la ignoro e la metto in closedlist. Se è libera la metto al fondo di openlist (da cui poi prendo e rimuovo il primo elemento come cella corrente al prossimo cico). Se è sconosciuta allora ho trovato una possibile frontiera e metto la cella correntemente analizzata in openfrontier. Allora scansiono le celle vicine ad essa alla ricerca di celle frontiera adiancenti e, se lo sono, le metto in un array cluster. Se sono un numero sufficiente per fungere da passaggio, allora calcolo il punto centrale nel cluster e lo restituisco. Se invece non sono abbastanza metto quella cella corrente in closedlist e continuo (era solo un errore della mappa o un passaggio stretto)
    // PROBLEMA: anche se il cluster di celle di frontiera è piccolo non significa che il robot non possa passarci, quindi potrebbe ancora essere una frontiera valida. Per risolvere potei cercare celle libere vicine a quella di frontiera. Se ci sono abbastanza celle libere (per esempio 4x4) e abbastanza celle di frontiera (che garantiscono che non sia solo un punto isolato) allora restituisco il punto medio delle celle di frontiera
    // Mettere anche un meccanismo per capire quando la mappa è completa

    uint8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    uint8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (uint8_t i = 0; i < 8; i++) {
        int16_t nX = currX + dX[i];
        int16_t nY = currY + dY[i];

        Pos chunkPos = Navigator::getChunkPos(nX, nY);
        int16_t index = Navigator::getPosIndex(nX, nY);

        if (_nav->getMap() > Navigator::THRESHOLD_OBSTACLE)
    }


}