#ifndef CONNECTION_H
#define CONNECTION_H

#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <map>
#include <set>
#include "Navigator.h"
#include "RobotMovements.h"
#include "../../../src/Common.h"

class Explorer;

enum MessTypeConn
{
    SET_TARGET = 0,
    START_EXPLORE = 1,
    STOP_EXPLORE = 2
};

class Connection
{

private:
    WebServer server;
    void handleMessage(MessTypeConn messageType, JsonVariant bodyMessage);
    void openResource(bool isRoot);
    void sendMap();
    const std::map<Pos, Chunk>* _map = nullptr;
    Navigator *_nav = nullptr;
    Explorer *_exp = nullptr;
    RobotMovements *_rb = nullptr;

    QueueHandle_t _messToClient;

public:
    Connection() : server(80) {}
    void init(Navigator &nav, Explorer &exp, RobotMovements &rb, QueueHandle_t messToClient);
    void update();
};

#endif