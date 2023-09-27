#pragma once

#include "ElectromagneticEntity.h"

#include <axmol.h>

#include <vector>


class MagnetEntity : public ElectromagneticEntity {
public:
	struct AttachedEntity {
		ax::Vec3 position;
		MagnetPolarity polarity;
	};

protected:
	std::vector<AttachedEntity> _attachedEntities;
	

public:
	virtual ~MagnetEntity() = default;

	
	std::vector<AttachedEntity>& getAttachedEntities() { return _attachedEntities; }

};
