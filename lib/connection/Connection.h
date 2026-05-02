#ifndef CONNECTION_H
#define CONNECTION_H

#include <Arduino.h>
#include <WebServer.h>
#include <map>
#include <set>
#include "Navigator.h"

enum MessType
{
    SET_TARGET,
    START_EXPLORE,
    STOP_EXPLORE
};

class Connection
{

private:
    WebServer server;
    void handleMessage(MessType messageType, JsonVariant bodyMessage);
    void openResource(bool isRoot);
    void sendMap();
    const std::map<Pos, Chunk>* _map = nullptr;
    Navigator *_nav;
    Explorer *_exp;

    bool _isExploring = false;

public:
    Connection() : server(80) {}
    void init(Navigator &nav, Explorer &exp);
    void update();
};

#endif