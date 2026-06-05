#include "Explorer.h"
#include "Navigator.h"

void Explorer::init(Navigator *n, RobotMovements *r, ServoMotor *s, LaserSensor *l, Ultrasonic *u, QueueHandle_t messToClient)
{
    _nav = n;
    _rb = r;
    _servo = s;
    _laser = l;
    _ultrasonic = u;

    _currServoP = _servo->getAngle();
}

void Explorer::setCurrentState(ExpState currState)
{
    _currentState = currState;
}

void Explorer::update()
{

    switch (_currentState)
    {
    case START_EXPLORING:
        _firstPos = _nav->getPos();
        _isExploring = true;
        _currentState = SCAN;
        break;

    case SCAN:
        scan();
        break;

    case MOVE_TO_FRONTIER:
    {
        Route routeFrontier = findBorder();
        if (routeFrontier.numSteps > 0)
        {
            _rb->setRoute(routeFrontier);
            _rb->setCurrentState(FOLLOWING);
            _currentState = FOLLOWING_F;
        }
        else
        {
            _isExploring = false;
            Route rHome = _nav->calcRoute(_firstPos.x, _firstPos.y);
            _currentState = COMPLETED;
            _rb->setRoute(rHome);
            _rb->setCurrentState(FOLLOWING);
        }
    }
    break;

    case FOLLOWING_F:
        if (_rb->getCurrentState() == COMPLETED_ROUTE) {
            _currentState = SCAN;
        }
        break;

    case COMPLETED:
        break;

    default:
        break;
    }
}

Pos Explorer::calcCoordinates(Pos currPos, float dis, int servoAngle)
{
    float rad = (servoAngle * PI) / 180.0f;
    float disX = cos(rad) * dis + (float)currPos.x;
    float disY = sin(rad) * dis + (float)currPos.y;

    int16_t nX = floor(disX / Navigator::CELL_CM);
    int16_t nY = floor(disY / Navigator::CELL_CM);

    return {nX, nY};
}

void Explorer::scan()
{
    // TODO tornare alla posizione iniziale del servo gradualmente
    _servo->moveToAngleFast(_currScanPoint);
    searchObstacles();
    _currScanPoint += SCAN_PRECISION;
    delay(30);

    if (ServoMotor::MAX_ANGLE - _servo->getAngle() < 2)
    {
        _currentState = MOVE_TO_FRONTIER;
        _currScanPoint = ServoMotor::MIN_ANGLE;
        return;
    }
}

void Explorer::searchObstacles()
{
    float laserDis = -1;
    float ultrasonicDis = -1;
    int servoAngle = _servo->getAngle();
    int robotAngle = _nav->getDir();
    int totAngle = servoAngle + robotAngle;

    Pos p = _nav->getPos();

    if (_laser->isReady())
    {
        laserDis = _laser->getDistance();
        Pos c = calcCoordinates(p, laserDis, totAngle);
        _nav->sculpt(c.x, c.y, Navigator::LASER);
    }

    ultrasonicDis = _ultrasonic->getDistance();
    Pos c = calcCoordinates(p, ultrasonicDis, totAngle);
    _nav->sculpt(c.x, c.y, Navigator::ULTRASONIC);
}

Route Explorer::findBorder()
{
    // Parte dalle coordinate del robot correnti e si espande in tutte le 8 celle adiacenti. Quindi con un for controllo singolarmente ogni cella adiacente alla cella correntemente analizzata. Se è un ostacolo la ignoro e la metto in closedlist. Se è libera la metto al fondo di openlist (da cui poi prendo e rimuovo il primo elemento come cella corrente al prossimo ciclo). Se è sconosciuta allora ho trovato una possibile frontiera e analizzo le celle adiacenti ad essa per capire se è rumore o è davvero un possibile passaggio. Se ci sono almeno altre 3 celle sconosciute adiacenti (3 sulle 8 adiacenti) allora calcolo la route fino a quel punto e restituisco. Se sono meno di 3 o se il percorso non è utilizzabile continuo a cercare

    int16_t maxExp = 7000; // Massime celle analizzate
    int16_t counterMax = 0;

    int16_t currX = _nav->getPos().x;
    int16_t currY = _nav->getPos().y;

    std::deque<Pos> mapOpenList;
    std::set<Pos> mapClosedList;

    bool foundFrontier = false;
    Pos foundFrPos = {0, 0};

    int8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    mapOpenList.push_front({currX, currY}); // Inserisco come primo valore la posizione corrente
    mapClosedList.insert({currX, currY});

    while (!mapOpenList.empty())
    {

        if (xSemaphoreTake(_nav->getMutex(), portMAX_DELAY) == pdTRUE)
        {

            const auto &map = _nav->getMap();

            if (counterMax > maxExp)
            {
                xSemaphoreGive(_nav->getMutex());

                Route emptyRoute;
                return emptyRoute;
            }
            counterMax++;

            Pos curr = mapOpenList.front();
            mapOpenList.pop_front();

            for (uint8_t i = 0; i < 8; i++)
            {
                int16_t nX = curr.x + dX[i]; // Coordinate logiche correnti
                int16_t nY = curr.y + dY[i];

                if (mapClosedList.find({nX, nY}) != mapClosedList.end())
                { // Se l'ho già vista e scartata passo oltre
                    continue;
                }

                Pos chunkPos = Navigator::getChunkPos(nX, nY);
                int16_t index = Navigator::getPosIndex(nX, nY);

                auto it = map.find(chunkPos);
                if (it != map.end())
                {
                    const Chunk &chunk = it->second; // Chunk in cui mi trovo virtualmente

                    if (chunk.cells[index] > Navigator::THRESHOLD_OBSTACLE)
                    { // Se è un ostacolo lo metto in closedlist
                        mapClosedList.insert({nX, nY});
                    }
                    else if (chunk.cells[index] < Navigator::THRESHOLD_OBSTACLE)
                    { // Se è libero va in openlist
                        mapOpenList.push_back({nX, nY});
                        mapClosedList.insert({nX, nY});
                    }
                    else if (chunk.cells[index] == DEFAULT_VAL)
                    { // Trovata possibile frontiera

                        uint8_t countFrontier = 0;
                        for (uint8_t j = 0; j < 8; j++)
                        { // Analizzo celle adiacenti a quella sconosciuta
                            int16_t nXC = nX + dX[j];
                            int16_t nYC = nY + dY[j];

                            Pos chunkPosC = Navigator::getChunkPos(nXC, nYC);
                            int16_t indexC = Navigator::getPosIndex(nXC, nYC);

                            auto itC = map.find(chunkPosC);
                            if (itC != map.end())
                            {
                                const Chunk &chunkC = itC->second;

                                if (chunkC.cells[indexC] == DEFAULT_VAL)
                                {
                                    countFrontier++; // Trovata una'altra frontiera adiacente
                                }
                            }
                        }

                        if (countFrontier >= 3)
                        {
                            // Se ci sono almeno 3 celle sconosciute adiacenti è una frontiera, restituisco il percorso
                            // Non posso chiamare qui calcRoute perché chiamerebbe aStar che chiamerebbe isFree che usa il mutex, causando deadlock
                            foundFrPos = {nX, nY};
                            foundFrontier = true;
                            break;
                        }
                    }
                }
                else
                {
                    // anche il chunk è sconosciuto, sicuramente è una frontiera
                    foundFrPos = {curr.x, curr.y};
                    foundFrontier = true;
                }
            }

            xSemaphoreGive(_nav->getMutex());
        }

        if (foundFrontier)
        {
            Route r = _nav->calcRoute(foundFrPos.x, foundFrPos.y);

            if (r.numSteps != -1)
            {
                return r;
            }
            else
            {
                foundFrontier = false;
                mapClosedList.insert(foundFrPos); // Non c'è un percorso utilizzabile, cerco altre frontiere
            }
        }
    }

    Route emptyRoute;
    return emptyRoute;
}