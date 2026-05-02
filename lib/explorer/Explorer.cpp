#include "Explorer.h"
#include "Navigator.h"

void Explorer::explore(Navigator &nav)
{
    _nav = &nav;
    _servo.init();
    _ultrasonic.init();
    _laser.init();

    Pos firstPos = _nav->getPos();

    scan();
    _rb.turn(180);
    scan();

    bool expCompleted = false;
    while (!expCompleted) {
        Route routeFrontier = findBorder();
        if (routeFrontier.numSteps > 0) {
            _rb.followPath(routeFrontier, *_nav);
            scan();
        } else {
            expCompleted = true;
            Route rHome = _nav->calcRoute(firstPos.x, firstPos.y);
            _rb.followPath(rHome, *_nav);
        }
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

Route Explorer::findBorder()
{

    int16_t maxExp = 7000;
    int16_t counterMax = 0;

    int16_t currX = _nav->getPos().x;
    int16_t currY = _nav->getPos().y;

    std::deque<Pos> mapOpenList;
    std::deque<Pos> openFrontier;
    std::set<Pos> mapClosedList;

    const auto& map = _nav->getMap();    

    uint8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    uint8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    mapOpenList.push_front({ currX, currY }); // Inserisco come primo valore la posizione corrente
    mapClosedList.insert({ currX, currY });

    while (!mapOpenList.empty()) {
        if (counterMax > maxExp) {
            Route emptyRoute;
            return emptyRoute;
        }
        counterMax++;

        Pos curr = mapOpenList.front();
        mapOpenList.pop_front();

        for (uint8_t i = 0; i < 8; i++) {
            int16_t nX = curr.x + dX[i]; // Coordinate logiche correnti
            int16_t nY = curr.y + dY[i];

            if (mapClosedList.find({nX, nY}) != mapClosedList.end()) { // Se l'ho già vista e scartata passo oltre
                continue;
            }

            Pos chunkPos = Navigator::getChunkPos(nX, nY);
            int16_t index = Navigator::getPosIndex(nX, nY);

            auto it = map.find(chunkPos);
            if (it != map.end()) {
                const Chunk& chunk = it->second; // Chunk in cui mi trovo

                if (chunk.cells[index] > Navigator::THRESHOLD_OBSTACLE) { // Se è un ostacolo lo metto in closedlist
                    mapClosedList.insert({ nX, nY });

                } else if (chunk.cells[index] < Navigator::THRESHOLD_OBSTACLE) { // Se è libero va in openlist
                    mapOpenList.push_back({nX, nY});
                    mapClosedList.insert({nX, nY});

                } else if (chunk.cells[index] == DEFAULT_VAL) { // Trovata possibile frontiera

                    uint8_t countFrontier = 0;
                    for (uint8_t j = 0; j < 8; j++) { // Analizzo celle adiacenti a quella sconosciuta
                        int16_t nXC = nX + dX[j];
                        int16_t nYC = nY + dY[j];

                        Pos chunkPosC = Navigator::getChunkPos(nXC, nYC);
                        int16_t indexC = Navigator::getPosIndex(nXC, nYC);

                        auto itC = map.find(chunkPosC);
                        if (itC != map.end()) {
                            const Chunk& chunkC = itC->second;

                            if (chunkC.cells[indexC] == DEFAULT_VAL) {
                                countFrontier++; // Trovata una'altra frontiera adiacente
                            }
                        }
                    }

                    if (countFrontier >= 3) { // Se ci sono almeno 3 celle sconosciute adiacenti è una frontiera, restituisco il percorso
                        Route r = _nav->calcRoute(nX, nY);

                        if (r.numSteps == -1) {
                            mapClosedList.insert({ nX, nY });
                            continue;
                        } else {
                            return r;
                        }
                    }

                }
            }

        }
    }

    Route emptyRoute;
    return emptyRoute;

}