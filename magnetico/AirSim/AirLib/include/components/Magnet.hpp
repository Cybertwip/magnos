#pragma once

#include "components/entities/MagnetEntity.hpp"

#include "components/entities/Node.hpp"

class Magnet : public msr::airlib::Node {
public:
	Magnet(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index);
	
	void updatePositions();
	
private:
	std::vector<MagnetEntity::AttachedEntity>& attachedEntities;
	size_t entityIndex;
};
