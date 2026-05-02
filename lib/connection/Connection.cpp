#include "Connection.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Navigator.h>
#include <WiFi.h>
#include <set>
#include <Explorer.h>

void Connection::handleMessage(MessType messageType, JsonVariant bodyMessage)
{

	switch (messageType)
	{
		case SET_TARGET:
			if (bodyMessage["x"].is<int>() && bodyMessage["y"].is<int>())
			{
				_nav->setDestination(bodyMessage["x"], bodyMessage["y"]);
				server.send(200, "text/plain", "Received");
			}
			else
			{
				server.send(200, "text/plain", "SET_TARGET error");
			}
			break;

		case START_EXPLORE:
			if (!_isExploring)
			{
				_exp->explore(*_nav);
				server.send(200, "text/plain", "Received");
				_isExploring = true;
			}
			
			break;

		case STOP_EXPLORE:
			if (_isExploring)
			{
				_exp->stopExploring();
				server.send(200, "text/plain", "Received");	
				_isExploring = false;
			}

			break;
		}
}

void Connection::init(Navigator &nav, Explorer &exp)
{
	// TODO usare websocket

	_map = &nav.getMap();
	_nav = &nav;
	_exp = &exp;

	if (!LittleFS.begin())
	{
		Serial.println("LittleFS error");
		return;
	}

	server.on("/api/getMap", [this]()
			  { this->sendMap(); });
	server.on("/", [this]()
			  { this->openResource(true); });

	server.on("/api/listen", HTTP_POST, [this]()
			  {
		if (server.hasArg("plain")) {
			String body = server.arg("plain");

			// {
			//     "messageType": "SET_TARGET" o "START_EXPLORE" o
			//     "STOP_EXPLORE", "body": ".."
			// }

			JsonDocument doc;
			DeserializationError error = deserializeJson(doc, body);

			if (error) {
				server.send(400, "text/plain", "Corrupt JSON");
				return;
			}

			MessType messageType = doc["messageType"];
			JsonVariant bodyMessage = doc["body"];

			handleMessage(messageType, bodyMessage);
		} else {
			server.send(400, "text/plain", "Body missing");
		} });

	server.onNotFound([this]()
					  { this->openResource(false); });

	server.begin();
}

void Connection::sendMap() {
    if (_map == nullptr) return;

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");
    server.sendContent("[");

    bool first = true;
    for (const auto& pair : *_map) {
        const Pos& chunkPos = pair.first;
        const Chunk& chunk = pair.second;

        for (int i = 0; i < 256; i++) {
            uint8_t cellValue = chunk.cells[i];

            if (cellValue != DEFAULT_VAL) {
                if (!first) server.sendContent(",");

                int16_t globalX = (chunkPos.x * 16) + (i % 16);
                int16_t globalY = (chunkPos.y * 16) + (i / 16);

                char buffer[48];
                snprintf(buffer, sizeof(buffer), "{\"x\":%d,\"y\":%d,\"v\":%d}", globalX, globalY, cellValue);
                server.sendContent(buffer);
                
                first = false;
            }
        }
    }
    server.sendContent("]");
}

void Connection::update() { server.handleClient(); }

void Connection::openResource(bool isRoot)
{
	String path = isRoot ? "/index.html" : server.uri();
	if (LittleFS.exists(path))
	{
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
	}
	else
	{
		server.send(404, "text/plain", "404 Not Found");
	}
}