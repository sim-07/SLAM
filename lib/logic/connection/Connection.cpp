#include "Connection.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Navigator.h>
#include <WiFi.h>
#include <set>
#include <Explorer.h>

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

	server.on("/api/sendMessage", HTTP_POST, [this]()
			  {
		if (server.hasArg("plain")) {
			String body = server.arg("plain");

			// {
			//     "messageType": "SET_TARGET" o "START_EXPLORE" o "STOP_EXPLORE", 
			//	   "body": ".."
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
			_exp->setCurrentState(START_EXPLORING);
			server.send(200, "text/plain", "Received");
		}

		break;

	case STOP_EXPLORE:
		if (_isExploring)
		{
			_exp->setCurrentState(COMPLETED);
			server.send(200, "text/plain", "Received");
		}

		break;
	}
}

void Connection::sendMap()
{

	server.setContentLength(CONTENT_LENGTH_UNKNOWN);
	server.send(200, "application/octet-stream", "");

	std::vector<Pos> positions;

	if (xSemaphoreTake(_nav->getMutex(), portMAX_DELAY) == pdTRUE)
	{
		for (const auto &pair : *_map)
		{
			positions.push_back(pair.first);
		}
		xSemaphoreGive(_nav->getMutex());
	}

	uint8_t tempCells[256];
	for (const auto &p : positions)
	{
		if (xSemaphoreTake(_nav->getMutex(), portMAX_DELAY) == pdTRUE)
		{

			auto it = _map->find(p);
			if (it != _map->end())
			{
				memcpy(tempCells, it->second.cells, 256);
			}
			else
			{
				xSemaphoreGive(_nav->getMutex());
				continue;
			}
			xSemaphoreGive(_nav->getMutex());
		}
		else
		{
			continue;
		}

		int16_t coords[2] = {p.x, p.y};
		server.sendContent((const char *)coords, sizeof(coords)); // Coordinate del Chunk: 4 byte, 2 per x e 2 per y
		server.sendContent((const char *)tempCells, 256);		  // invio contenuto binario dell'array coords. Char perché .sendContent accetta solo char, altrimenti sarebbe uguale
	}
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