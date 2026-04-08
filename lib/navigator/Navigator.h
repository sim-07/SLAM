#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <stack>
#include <utility>
#include <vector>
#include <map>

static const uint8_t MAP_WIDTH = 100;
static const uint8_t MAP_HEIGHT = 100;
static const uint8_t CHUNK_DIM = 256;
static const uint8_t CHUNK_WIDTH = sqrt(CHUNK_DIM);
static const uint8_t ULTRASONIC_A = 10;
static const uint8_t LASER_A = 20;
static const uint8_t N_MAP_V = 128;
static const uint8_t THRESHOLD_OBSTACLE = N_MAP_V - LASER_A - ULTRASONIC_A;

struct Pos
{
	int16_t x, y;

	bool operator<(const Pos& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

struct ChunkPos
{
	int16_t x, y;

	bool operator<(const ChunkPos &other) const
	{
		if (x != other.x)
			return x < other.x;
		return y < other.y;
	}
};

struct Chunk
{
	uint8_t cells[CHUNK_DIM];

	Chunk()
	{
		memset(cells, N_MAP_V, CHUNK_DIM);
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
	std::map<ChunkPos, Chunk *> _obstacles;

	Pos _currPos;
	Pos _destination;
	float _currDir;

	bool isObstacle(int16_t x, int16_t y);
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

	const std::map<ChunkPos, Chunk *> &Navigator::getMap() const;
	float getDir();

	void setDir(float angle);
	void setDestination(int16_t x, int16_t y);

	void setCurrPos(int16_t x, int16_t y);
	void addObstacle(int16_t x, int16_t y, SensorType st);
};

#endif