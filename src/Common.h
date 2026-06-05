#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

enum MessType {
    INFO = 0,
    ERROR = 1
};

struct Message {
    MessType type;
    char mess[64]; // La versione corretta e sicura senza String!
};

#endif