#pragma once

#include "ElectromagneticEntity.hpp"

class CoilEntity : public ElectromagneticEntity {
public:
	struct AttachedEntity {
		msr::airlib::Vector3r position;
		MagnetPolarity polarity;
		float area = 0;  // Area of the coil.
		float turns = 0;  // Number of turns in the coil.
		float flux = 0;
		float previousFlux = 0;
	};
	
protected:
	std::vector<AttachedEntity> _attachedEntities;
	
	
public:
	virtual ~CoilEntity() = default;
	
	
	std::vector<AttachedEntity>& getAttachedEntities() { return _attachedEntities; }
	
};
