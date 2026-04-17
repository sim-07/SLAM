#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <stack>
#include <utility>
#include <vector>
#include <map>
#include <memory>

static const uint8_t MAP_WIDTH = 100;
static const uint8_t MAP_HEIGHT = 100;
static const uint8_t CHUNK_DIM = 256;
static const uint8_t CHUNK_WIDTH = sqrt(CHUNK_DIM);
static const uint8_t ULTRASONIC_A = 10;
static const uint8_t LASER_A = 20;
static const uint8_t BLANK_A = 20;
static const uint8_t DEFAULT_VAL = 128;
static const uint8_t THRESHOLD_OBSTACLE = DEFAULT_VAL - LASER_A;

struct Pos
{
	int16_t x, y;

	bool operator<(const Pos &other) const
	{
		if (x != other.x)
			return x < other.x;
		return y < other.y;
	}
};

struct Chunk {
    uint8_t cells[256]; // TODO: ogni cella ne continene 2. Doppie celle con la stessa memoria. 

    Chunk() {
        memset(cells, DEFAULT_VAL, sizeof(cells));
    }
};

struct Route
{
	std::stack<Pos> route;
	int numSteps;
};

struct Node
{
	int16_t parent_i, parent_j;

	uint16_t f, g,
		h;
	// g = dalla partenza alla cella attuale, h costo stimato da cella
	// attuale alla destinazione, f è la somma di g e h
};

class Navigator
{
	// matrice virtuale, salvo solo gli ostacoli in coordinate immaginarie
private:
	std::map<Pos, Chunk> _map;

	Pos _currPos;
	Pos _destination;
	float _currDir;

	void createBlanks(int16_t targetX, int16_t targetY);
	bool isFree(int16_t x, int16_t y);
	Route aStar(Pos start, Pos goal);

public:
	enum SensorType
	{
		ULTRASONIC = 0,
		LASER = 1
	};

	Navigator();

	Pos getPos() { return _currPos; }
	Route calcRoute(int16_t x, int16_t y);
	double calcDistanceBetween(Pos start, Pos dest);
	int16_t getChunkIndex(int16_t x, int16_t y);
	Pos getChunkPos(int16_t x, int16_t y);

	const std::map<Pos, Chunk> &Navigator::getMap() const;
	float getDir();

	void setDir(float angle);
	void setDestination(int16_t x, int16_t y);
	void setCurrPos(int16_t x, int16_t y);
	void sculpt(int16_t targetX, int16_t targetY, SensorType st);
};

#endif