#include <string.h>
#include <math.h>
#include <float.h>
#include <set>

#include "Navigator.h"
#include <stack>
#include <map>

using namespace std;

const int ULTRASONIC_A = 10;
const int LASER_A = 20;
const int MAX_MAP_V = 128;
const int THRESHOLD_OBSTACLE = MAX_MAP_V - LASER_A;

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

void Navigator::setDestination(int x, int y)
{
    _destination = {x, y};
}

void Navigator::setCurrPos(int x, int y)
{
    _currPos = {x, y};
}

bool Navigator::isObstacle(int x, int y)
{
    return _obstacles.count({x, y});
}

const std::set<Pos>& Navigator::getMap() const {
    return _obstacles;
}

void Navigator::addObstacle(int x, int y, SensorType st)
{

    int v = 0;
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

    int dX[] = {-1, 1, 0, 0, 0, -1, -1, 1, 1};
    int dY[] = {0, 0, 1, -1, 0, 1, -1, 1, -1};

    int padding = 9;
    for (int k = 0; k < padding; k++)
    {
        int nRow = x + dX[k];
        int nCol = y + dY[k];

        _obstacles.insert({nRow, nCol});
    }
}

Route Navigator::calcRoute(int dest_x, int dest_y)
{
    Pos destPos = {dest_x, dest_y};

    return aStar(_currPos, destPos);
}

///////////////////////// A* /////////////////////////

bool isDestination(int row, int col, Pos dest)
{
    if (row == dest.x && col == dest.y)
        return true;
    else
        return false;
}

double calculateHValue(int row, int col, Pos dest)
{
    return ((double)sqrt(
        (row - dest.x) * (row - dest.x) + (col - dest.y) * (col - dest.y)));
}

Route tracePath(std::map<Pos, Node> &cellDetails, Pos dest)
{
    // TODO ottimizzare lista ostacoli raggruppando quelli vicini
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
    std::set<Pos> closedList;                     // celle esplorate

    openList.insert({0, {currRow, currCol}}); // fNew, {x, y}
    cellDetails[{currRow, currCol}] = {currRow, currCol, 0, 0, 0};
    openList.insert({0, {currRow, currCol}});

    int dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

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
            int nRow = currRow + dX[k];
            int nCol = currCol + dY[k];

            auto it = cellDetails.find({nRow, nCol});

            if (isDestination(nRow, nCol, {goal.x, goal.y}))
            {
                // se la prossima cella (nRow e nCol) è destinazione, metto i parent della prossima cella con i valori row e col della cella corrente e return
                cellDetails[{nRow, nCol}].parent_i = currRow;
                cellDetails[{nRow, nCol}].parent_j = currCol;

                return tracePath(cellDetails, {nRow, nCol});
            }

            if (!closedList.count({nRow, nCol}) && !isObstacle(nRow, nCol))
            {
                // se non è la meta controllo che non sia già stato visto e che non sia un ostacolo
                uint16_t cost = (k < 4) ? 10 : 14; // se è diagonale costerà 1.41, gli ultimi 4 k sono diagonali
                uint16_t gNew = cellDetails[{currRow, currCol}].g + cost;
                uint16_t hNew = calculateHValue(nRow, nCol, {goal.x, goal.y}) * 10;
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