#include "WifiConn.h"
#include <WiFi.h>

const char *ssid = "PicoW";
const char *password = "slamtest";

void WifiConn::init() {
	WiFi.mode(WIFI_AP);

	if (WiFi.softAP(ssid, password, _channel, _hidden, _maxConn)) {
		Serial.print("IP: ");
		Serial.println(WiFi.softAPIP());
	} else {
		Serial.println("Error Access Point");
	}
}