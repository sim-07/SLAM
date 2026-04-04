#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <vector>
#include <utility>
#include <stack>

static const int MAP_WIDTH = 100;
static const int MAP_HEIGHT = 100;

struct Pos {
    int x;
    int y;    

    bool operator<(const Pos& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

struct Route {
    stack<Pos> route;
    int numSteps;
};

struct Node {
    int16_t parent_i, parent_j;

    uint16_t f, g, h; // g = dalla partenza alla cella attuale, h costo stimato da cella attuale alla destinazione, f è la somma di g e h
};

enum SensorType {
    ULTRASONIC = 0,
    LASER = 1
};

class Navigator {
    // matrice virtuale, salvo solo gli ostacoli in coordinate fittizie
    private:
        std::set<Pos> _obstacles;

        Pos _currPos;
        Pos _destination;
        float _currDir;
        
        bool isObstacle(int x, int y);
        Route aStar(Pos start, Pos goal);

    public:
        Navigator();

        Pos getPos() { return _currPos; }
        Route calcRoute(int x, int y);

        float getDir();
        
        void setDir(float angle);
        void setDestination(int x, int y);
        void setCurrPos(int x, int y);
        void addObstacle(int x, int y, SensorType st);

};

#endif