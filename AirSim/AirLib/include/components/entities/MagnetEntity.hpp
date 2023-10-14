#pragma once

#include "components/entities/ElectromagneticEntity.hpp"

#include "common/Common.hpp"

class MagnetEntity : public ElectromagneticEntity {
public:
	struct AttachedEntity {
		msr::airlib::Vector3r position;
		MagnetPolarity polarity;
	};

protected:
	std::vector<AttachedEntity> _attachedEntities;
	

public:
	virtual ~MagnetEntity() = default;

	
	std::vector<AttachedEntity>& getAttachedEntities() { return _attachedEntities; }

};
