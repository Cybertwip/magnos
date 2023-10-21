#pragma once

#include "entities/CoilEntity.hpp"

#include "components/entities/Node.hpp"

class Coil : public msr::airlib::Node {
public:
	Coil(std::vector<CoilEntity::AttachedEntity>& entities, size_t index);	
	void updatePositions();
	
private:
	std::vector<CoilEntity::AttachedEntity>& attachedEntities;
	size_t entityIndex;
};
