#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <vector>

static const int MAP_WIDTH = 100;
static const int MAP_HEIGHT = 100;
static const int MAX_STEPS = 200;

struct Pos {
    int x;
    int y;
    int dir;
};

struct Route {
    Pos route[MAX_STEPS]; // tappe max
    int numSteps;
};

struct Node {
    int8_t parent_i, parent_j;

    float f, g, h;
};


class Navigator {

    private:
        byte _map[MAP_WIDTH][MAP_HEIGHT]; // 10cm per cell
        Pos _currPos;
        Pos _destination;

        std::vector<Pos> aStar(Pos start, Pos goal);

    public:
        Navigator();

        Pos getPos() { return _currPos; }

        Route calcRoute(int x, int y, int dir);

        void setDestination(int x, int y);
        void setCurrPos(int x, int y, int dir);
        void sculpt(int x, int y, byte val);

};

#endif