#ifndef WIFI_H
#define WIFI_H

#include <Arduino.h>

class WifiConn {

  private:
	const int _channel = 1;
	const bool _hidden = false;
	const int _maxConn = 1;

  public:
	void init();
	// void update();
};

#endif