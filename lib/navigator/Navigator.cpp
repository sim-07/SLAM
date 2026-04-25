#include <string.h>
#include <math.h>
#include <float.h>
#include <set>

#include "Navigator.h"
#include <stack>
#include <map>

using namespace std;

Navigator::Navigator()
{
    _currPos = {0, 0};
}

float Navigator::getDir()
{
    return _currDir;
}

void Navigator::setDir(float angle)
{
    _currDir = angle;
}

void Navigator::setDestination(int16_t x, int16_t y)
{
    _destination = {x, y};
}

void Navigator::setCurrPos(int16_t x, int16_t y)
{
    _currPos = {x, y};
}

bool Navigator::isFree(int16_t x, int16_t y)
{
    Pos p = getChunkPos(x, y);
    auto it = _map.find(p);

    if (it == _map.end()) {
        return false;
    }

    int16_t cellIndex = getPosIndex(x, y);

    return (it->second.cells[cellIndex] < THRESHOLD_OBSTACLE);
}

const std::map<Pos, Chunk> &Navigator::getMap() const
{
    return _map;
}

Pos Navigator::getChunkPos(int16_t x, int16_t y)
{
    return {x >> 4, y >> 4};
}

int16_t Navigator::getPosIndex(int16_t x, int16_t y)
{
    int16_t posCellX = x & 0x0F; // % 16
    int16_t posCellY = y & 0x0F;

    return (posCellY * CHUNK_WIDTH) + posCellX;
}

void Navigator::sculpt(int16_t targetX, int16_t targetY, SensorType st)
{

    uint16_t v = 0;
    switch (st)
    {
    case ULTRASONIC:
        v = ULTRASONIC_A;
        break;

    case LASER:
        v = LASER_A;
        break;

    default:
        break;
    }

    createBlanks(targetX, targetY);

    int dX[] = {-1, 1, 0, 0, 0, -1, -1, 1, 1};
    int dY[] = {0, 0, 1, -1, 0, 1, -1, 1, -1};

    int padding = 9;
    for (int k = 0; k < padding; k++)
    {
        int16_t nX = targetX + dX[k];
        int16_t nY = targetY + dY[k];

        Pos cPos = getChunkPos(nX, nY);
        int16_t cellIndex = getPosIndex(nX, nY);

        Chunk &currChunk = _map[cPos];

        if (currChunk.cells[cellIndex] + v > 255)
        {
            currChunk.cells[cellIndex] = 255;
        }
        else
        {
            currChunk.cells[cellIndex] += v;
        }
    }
}

Route Navigator::calcRoute(int16_t dest_x, int16_t dest_y)
{
    Pos destPos = {dest_x, dest_y};

    return aStar(_currPos, destPos);
}

// Bresenham
void Navigator::createBlanks(int16_t targetX, int16_t targetY)
{
    // TODO potrebbe essere necessario mettere un padding anche agli spazi liberi
    Pos chunkCurrPos = getChunkPos(getPos().x, getPos().y);
    Pos chunkDestPos = getChunkPos(targetX, targetY);
    int16_t cellIndex = getPosIndex(targetX, targetY);

    int16_t x0 = getPos().x;
    int16_t y0 = getPos().y;
    int16_t x1 = targetX;
    int16_t y1 = targetY;

    // delta x e y
    int16_t dX = std::abs(x1 - x0);
    int16_t dY = -std::abs(y1 - y0);

    // direzione destinazione
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;

    // err serve per decidere in quale direzione muoversi
    int16_t err = dX + dY;

    Pos lastPos = {0, 0};
    Chunk *currChunk = nullptr;

    while (true)
    {
        // esco prima della destinazione, non voglio resettare l'ultimo chunk
        if (x0 == x1 && y0 == y1)
            break;

        Pos cPos = getChunkPos(x0, y0);

        if (currChunk == nullptr || cPos.x != lastPos.x || cPos.y != lastPos.y)
        {
            // riprendo il chunk dal map solo se è cambiato o è nullptr (nel caso in cui coordinate fossero effettivamente 0, 0)
            currChunk = &_map[cPos];
            lastPos = cPos;
        }

        int16_t cellIndex = getPosIndex(x0, y0);

        if (currChunk->cells[cellIndex] < BLANK_A)
        {
            currChunk->cells[cellIndex] = 0;
        }
        else
        {
            currChunk->cells[cellIndex] -= BLANK_A;
        }

        int e2 = 2 * err;

        if (e2 >= dY)
        {
            err += dY;
            x0 += sx;
        }

        if (e2 <= dX)
        {
            err += dX;
            y0 += sy;
        }
    }
}

///////////////////////// A* /////////////////////////

bool isDestination(int16_t row, int16_t col, Pos dest)
{
    if (row == dest.x && col == dest.y)
        return true;
    else
        return false;
}

double Navigator::calcDistanceBetween(Pos start, Pos dest)
{
    return ((double)sqrt(
        (start.x - dest.x) * (start.x - dest.x) + (start.x - dest.y) * (start.y - dest.y)));
}

Route tracePath(std::map<Pos, Node> &cellDetails, Pos dest)
{
    Route r;

    r.numSteps = 0;

    int16_t row = dest.x;
    int16_t col = dest.y;

    float curr_angle;

    while (true)
    {
        auto it = cellDetails.find({row, col});

        if (it == cellDetails.end())
            break;

        Pos p = {row, col};
        r.route.push(p);
        r.numSteps++;

        if (it->second.parent_i == row && it->second.parent_j == col) // la cella iniziale è l'unica ad avere i parents con le stesse proprie coordinate
        {
            break;
        }

        row = it->second.parent_i;
        col = it->second.parent_j;
    }

    return r;
}

const uint16_t MAX_V = 65535;
Route Navigator::aStar(Pos start, Pos goal)
{

    // cella di partenza
    int16_t currRow = start.x;
    int16_t currCol = start.y;

    std::map<Pos, Node> cellDetails;              // informazioni sulle celle visitate (coordinate, g, f, h, parent)
    set<pair<uint16_t, pair<int, int>>> openList; // celle da esplorare ordinate in base alla f
    std::set<Pos> closedList;                     // celle già esplorate

    openList.insert({0, {currRow, currCol}}); // fNew, {x, y}
    cellDetails[{currRow, currCol}] = {currRow, currCol, 0, 0, 0};
    openList.insert({0, {currRow, currCol}});

    uint8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    uint8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!openList.empty())
    {
        auto p = *openList.begin();
        openList.erase(openList.begin());

        currRow = p.second.first;  // x
        currCol = p.second.second; // y
        closedList.insert({currRow, currCol});

        for (int k = 0; k < 8; k++)
        {
            // vengono controllate tutte le 8 direzioni
            int16_t nRow = currRow + dX[k];
            int16_t nCol = currCol + dY[k];

            auto it = cellDetails.find({nRow, nCol});

            if (isDestination(nRow, nCol, {goal.x, goal.y}))
            {
                // se la prossima cella (nRow e nCol) è destinazione, metto i parent della prossima cella con i valori row e col della cella corrente e return
                cellDetails[{nRow, nCol}].parent_i = currRow;
                cellDetails[{nRow, nCol}].parent_j = currCol;

                return tracePath(cellDetails, {nRow, nCol});
            }

            if (!closedList.count({nRow, nCol}) && isFree(nRow, nCol))
            {
                // se non è la meta controllo che non sia già stato visto e che non sia un ostacolo
                uint16_t cost = (k < 4) ? 10 : 14; // se è diagonale costerà 1.41, gli ultimi 4 k sono diagonali
                uint16_t gNew = cellDetails[{currRow, currCol}].g + cost;
                uint16_t hNew = calcDistanceBetween({nRow, nCol}, {goal.x, goal.y}) * 10;
                uint16_t fNew = gNew + hNew;

                if (it == cellDetails.end() || it->second.f > fNew)
                {
                    cellDetails[{nRow, nCol}] = {
                        currRow,
                        currCol,
                        fNew,
                        gNew,
                        hNew};

                    openList.insert({fNew, {nRow, nCol}}); // aggiungo il prossimo valore alla lista delle celle da esplorare
                }
            }
        }
    }

    return Route();
}