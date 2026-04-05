#ifndef EXPLORER_H
#define EXPLORER_H

#include "Navigator.h"

class Explorer {

    private:
        void setObstacle(Pos p);

    public:
        void explore(Navigator nav);
        void stopExploring();

};

#endif