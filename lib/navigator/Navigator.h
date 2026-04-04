#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <vector>
#include <utility>
#include <stack>

typedef std::pair<int, int> Pair;
typedef std::pair<double, std::pair<int, int>> pPair;

static const int MAP_WIDTH = 100;
static const int MAP_HEIGHT = 100;

struct Pos {
    int x;
    int y;
};

struct Route {
    stack<Pos> route;
    int numSteps;
};

struct Node {
    int8_t parent_i, parent_j;

    uint16_t f, g, h;
};

enum SensorType {
    ULTRASONIC = 0,
    LASER = 1
};

enum CellType {
    BLOCKED = 0,
    FREE = 1
};


class Navigator {

    private:
        uint8_t _map[MAP_WIDTH][MAP_HEIGHT]; // 15cm per cell -- TODO map virtuale, salvare solo coordinate con ostacoli in un set
        Pos _currPos;
        Pos _destination;
        float _currDir;

        Route aStar(Pos start, Pos goal);

    public:
        Navigator();

        Pos getPos() { return _currPos; }
        Route calcRoute(int x, int y);

        float getDir();
        
        void setDir(float angle);
        void setDestination(int x, int y);
        void setCurrPos(int x, int y);
        void sculpt(int x, int y, SensorType st, CellType c);

};

#endif