#ifndef CONNECTION_H
#define CONNECTION_H

#include <Arduino.h>
#include <WebServer.h>

enum MessType {
    SET_TARGET,
    START_EXPLORE,
    STOP_EXPLORE
};

class Connection {

  private:
	WebServer server;

	void openResource(bool isRoot);
    void sendMap();

    const std::set<Pos>* _map = nullptr;

  public:
	Connection() : server(80) {}
	void init(const std::set<Pos> &map);
	void update();
};

#endif