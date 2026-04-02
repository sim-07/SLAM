#include <string.h>

#include "Navigator.h"

Navigator::Navigator() {
    memset(_map, 128, sizeof(_map));
    _currPos = {0, 0, 0};
}

void Navigator::setDestination(int x, int y) {
    _destination = {x, y};
}

void Navigator::setCurrPos(int x, int y, int dir) {
    _currPos = {x, y, dir};
}

void Navigator::sculpt(int x, int y, byte val) {

    if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
        int newVal = _map[x][y] - val; // probabilità che si sia ostacolo o libero. Ostacolo -, libero +. Inizialmente 128

        _map[x][y] = (byte)constrain(newVal, 0, 255);
    }

}

Route Navigator::calcRoute(int x, int y, int dir) {
    Route r;
    r.numSteps = 0;

    Pos pos = {x, y, dir};

    std::vector<Pos> path = aStar(pos, _destination);

    int n = path.size();

    for (int i = 0; i < n && i < MAX_STEPS; i++) {
        r.route[i].x = path[(n - 1) - i].x;
        r.route[i].y = path[(n - 1) - i].y;
        r.route[i].dir = 0; // TODO calcolare direzione corretta
        
        r.numSteps++;
    }

    return r;
}

std::vector<Pos> Navigator::aStar(Pos start, Pos destination) {

}
