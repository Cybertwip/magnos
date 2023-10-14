#pragma once 
#include "common/Common.hpp"

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
	virtual msr::airlib::Vector3r combineFieldsOrForces(const msr::airlib::Vector3r& origin) = 0;	
	virtual ~ElectromagneticEntity() = default;

};
