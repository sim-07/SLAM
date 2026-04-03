#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <vector>
#include <utility>

typedef std::pair<int, int> Pair;
typedef std::pair<double, std::pair<int, int>> pPair;

static const int MAP_WIDTH = 100;
static const int MAP_HEIGHT = 100;

struct Pos {
    int x;
    int y;
    int dir;
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
        byte _map[MAP_WIDTH][MAP_HEIGHT]; // 10cm per cell
        Pos _currPos;
        Pos _destination;

        Route aStar(Pos start, Pos goal);

    public:
        Navigator();

        Pos getPos() { return _currPos; }

        Route calcRoute(int x, int y, int dir);

        void setDestination(int x, int y);
        void setCurrPos(int x, int y, int dir);
        void sculpt(int x, int y, SensorType st, CellType c);

};

#endif