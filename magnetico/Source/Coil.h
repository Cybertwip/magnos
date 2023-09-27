#pragma once

#include "CoilEntity.h"

#include <axmol.h>

#include <vector>

class Coil : public ax::Node {
public:
	Coil(std::vector<CoilEntity::AttachedEntity>& entities, size_t index);	
	static Coil* create(std::vector<CoilEntity::AttachedEntity>& entities, size_t index);
	void updatePositions();
	
private:
	std::vector<CoilEntity::AttachedEntity>& attachedEntities;
	size_t entityIndex;
};
