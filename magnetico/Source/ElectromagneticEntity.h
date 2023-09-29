#pragma once 

#include <axmol.h>

// Define direction flags
enum class MagnetDirection {
	NORTH,
	NORTHEAST,
	EAST,
	SOUTHEAST,
	SOUTH,
	SOUTHWEST,
	WEST,
	NORTHWEST,
	FRONT,
	BACK
};

enum class MagnetPolarity {
	NORTH,
	SOUTH
};

class ElectromagneticEntity {	
public:
	virtual ax::Vec3 combineFieldsOrForces() = 0;
	virtual void attachToDisk(ax::Node* disk, float radius, MagnetDirection direction, MagnetPolarity polarity) = 0;
	
	virtual ~ElectromagneticEntity() = default;

};
