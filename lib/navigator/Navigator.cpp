#include <string.h>
#include <math.h>
#include <float.h>
#include <set>

#include "Navigator.h"
#include <stack>

using namespace std;

const int ULTRASONIC_A = 10;
const int LASER_A = 20;
const int MAX_MAP_V = 128;
const int THRESHOLD_OBSTACLE = MAX_MAP_V - LASER_A;

Navigator::Navigator() {
    memset(_map, MAX_MAP_V, sizeof(_map));
    _currPos = {50, 50};
}

float Navigator::getDir(){
    return _currDir;
}

void Navigator::setDir(float angle) {
    _currDir = angle;
}

void Navigator::setDestination(int x, int y) {
    _destination = {x, y};
}

void Navigator::setCurrPos(int x, int y) {
    _currPos = {x, y};
}

void Navigator::sculpt(int x, int y, SensorType st, CellType c) {
    // probabilità che ci sia ostacolo o libero. Ostacolo -, libero +. Inizialmente 128. Quanto toglie dipende da affidabilità sensore

    int v = 0;
    int a = (c == BLOCKED) ? -1 : 1;

    switch (st) {
        case ULTRASONIC:
            v = ULTRASONIC_A * a;
            break;
        
        case LASER:
            v = LASER_A * a;
            break;
        
        default:
            break;
    }

    if (x >= 0 && x < MAP_HEIGHT && y >= 0 && y < MAP_WIDTH) {
        int dX[] = {-1, 1, 0, 0, 0, -1, -1, 1, 1};
        int dY[] = {0, 0, 1, -1, 0, 1, -1, 1, -1};

        int padding = (c == BLOCKED) ? 9 : 1;
        for (int k = 0; k < padding; k++) {
            int nRow = x + dX[k];
            int nCol = y + dY[k];

            if (nRow >= 0 && nRow < MAP_HEIGHT && nCol >= 0 && nCol < MAP_WIDTH) {
                int newVal = _map[nRow][nCol] + v; 
                _map[nRow][nCol] = (uint8_t)constrain(newVal, 0, 255);
            }
        
        }   
    }
}

Route Navigator::calcRoute(int dest_x, int dest_y) {
    Pos pos = { dest_x, dest_y };

    return aStar(pos, _destination);
}


///////////////////////// A* /////////////////////////

bool isValid(int row, int col)
{
    return (row >= 0) && (row < MAP_HEIGHT) && (col >= 0)
           && (col < MAP_WIDTH);
}

bool isUnBlocked(uint8_t grid[][MAP_WIDTH], int row, int col)
{
    return grid[row][col] >= THRESHOLD_OBSTACLE;
}

bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second)
        return true;
    else
        return false;
}

double calculateHValue(int row, int col, Pair dest)
{
    return ((double)sqrt(
        (row - dest.first) * (row - dest.first)
        + (col - dest.second) * (col - dest.second)));
}

Route tracePath(Node cellDetails[][MAP_WIDTH], Pair dest)
{
    Route r;
    
    r.numSteps = 0;

    int row = dest.first;
    int col = dest.second;

    float curr_angle;
    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col)) { // cella iniziale è l'unica ad avere se stessa come genitore

        int nextRow = cellDetails[row][col].parent_i;
        int nextCol = cellDetails[row][col].parent_j;

        Pos p;

        p.x = row;
        p.y = col;

        r.route.push(p);
        
        r.numSteps++;

        row = nextRow;
        col = nextCol;
    }

    return r;
}

// Prova tutte le 8 direzioni, mette i valori in celldetails e mette la f minore in openlist da cui poi riparte dopo, così fino alla destinazione
const uint16_t MAX_V = 65535;
Route Navigator::aStar(Pos start, Pos goal) {
    static Node cellDetails[MAP_WIDTH][MAP_HEIGHT];
    static bool closedList[MAP_WIDTH][MAP_HEIGHT];
    
    // pulizia memoria celldetails
    memset(closedList, false, sizeof(closedList));
    for (int i = 0; i < MAP_WIDTH; i++) {
        for (int j = 0; j < MAP_HEIGHT; j++) {
            cellDetails[i][j].f = MAX_V; // Valore "infinito" per uint16_t
            cellDetails[i][j].g = 0;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // cella di partenza
    int row = start.x;
    int col = start.y;
    cellDetails[row][col] = { (int8_t)row, (int8_t)col, 0, 0, 0 };

    set<pair<uint16_t, pair<int, int>>> openList;
    openList.insert({0, {row, col}});

    int dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!openList.empty()) {
        auto p = *openList.begin();
        openList.erase(openList.begin());

        row = p.second.first;
        col = p.second.second;
        closedList[row][col] = true;

        for (int k = 0; k < 8; k++) {
            int nRow = row + dX[k];
            int nCol = col + dY[k];

            if (isValid(nRow, nCol)) {
                if (isDestination(nRow, nCol, {goal.x, goal.y})) {
                    cellDetails[nRow][nCol].parent_i = row;
                    cellDetails[nRow][nCol].parent_j = col;
                    return tracePath(cellDetails, {nRow, nCol});
                }
                
                if (!closedList[nRow][nCol] && isUnBlocked(_map, nRow, nCol)) {

                    // Costo: 10 per i lati, 14 per diagonali (orizzontale e verticale costa 1, diagonale = radice 2 = 1.41 --> Tutto moltiplicato * 10 per farli numeri interi)
                    uint16_t cost = (k < 4) ? 10 : 14; // primi 4 movimenti sono dritti, altri diagonali
                    uint16_t gNew = cellDetails[row][col].g + cost;
                    uint16_t hNew = calculateHValue(nRow, nCol, {goal.x, goal.y}) * 10;
                    uint16_t fNew = gNew + hNew;

                    if (cellDetails[nRow][nCol].f == MAX_V || cellDetails[nRow][nCol].f > fNew) {
                        cellDetails[nRow][nCol].f = fNew;
                        cellDetails[nRow][nCol].g = gNew;
                        cellDetails[nRow][nCol].h = hNew;
                        cellDetails[nRow][nCol].parent_i = row;
                        cellDetails[nRow][nCol].parent_j = col;
                        openList.insert({fNew, {nRow, nCol}});
                    }
                }
            }
        }
    }

    return Route();
}