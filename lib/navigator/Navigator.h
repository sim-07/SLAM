#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>

struct Pos {
    int x;
    int y;
    int dir;
};

struct Route {
    Pos route[200]; // tappe max
    Pos nextHop;
};

enum NavStatus {
    SUCCESS = 0,
    OBSTACLE = 1,
    FAILED = 2
};

class Navigator {

    private:
        byte _map[100][100]; // 10cm per cell
        Pos _currPos;

    public:
        Pos getPos() { return _currPos; }
        NavStatus goTo(int x, int y, int dir);
        NavStatus followPath(Route &route);

        Route calcRoute(int x, int y, int dir);

        void setCurrPos(int x, int y, int dir);

        void setObstacle(int x, int y);
        void setFreeCell(int x, int y);

};

#endif