#include "Connection.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Navigator.h>
#include <WiFi.h>
#include <set>

void Connection::init(const std::set<Pos> &map) {
	_map = &map;
	if (!LittleFS.begin()) {
		Serial.println("LittleFS error");
		return;
	}

	server.on("/api/getMap", [this]() { this->sendMap(); });
	server.on("/", [this]() { this->openResource(true); });

	server.on("/api/listen", HTTP_POST, [this]() {
		if (server.hasArg("plain")) {
			String body = server.arg("plain");

			// {
			//     "messageType": "SET_TARGET" o "START_EXPLORE" o "STOP_EXPLORE",
            //     "body": ".."
			// }

			JsonDocument doc;
			DeserializationError error = deserializeJson(doc, body);

			if (error) {
				server.send(400, "text/plain", "Corrupt JSON");
				return;
			}

            MessType messageType = doc["messageType"];

            switch (messageType) {
                case SET_TARGET:
                    // TODO
                    break;
                case START_EXPLORE:
                    //TODO
                    break;
                case STOP_EXPLORE:
                    //TODO
                    break;
            }

			server.send(200, "text/plain", "OK");
		} else {
			server.send(400, "text/plain", "Body missing");
		}
	});

	server.onNotFound([this]() { this->openResource(false); });

	server.begin();
}

void Connection::sendMap() {
	if (_map == nullptr) {
		server.send(500, "text/plain", "Map error");
		return;
	}

	String jsonMap = "[";
	for (const auto &p : *_map) {
		jsonMap += "{\"x\":" + String(p.x) + ",\"y\":" + String(p.y) + "},";
	}
	if (jsonMap.endsWith(","))
		jsonMap.remove(jsonMap.length() - 1);
	jsonMap += "]";

	server.send(200, "application/json", jsonMap);
}

void Connection::update() { server.handleClient(); }

void Connection::openResource(bool isRoot) {
	String path = isRoot ? "/index.html" : server.uri();
	if (LittleFS.exists(path)) {
		String contentType = "text/plain";

		if (path.endsWith(".html"))
			contentType = "text/html";
		else if (path.endsWith(".css"))
			contentType = "text/css";
		else if (path.endsWith(".js"))
			contentType = "application/javascript";

		File file = LittleFS.open(path, "r");
		server.streamFile(file, contentType);
		file.close();
	} else {
		server.send(404, "text/plain", "404 Not Found");
	}
}