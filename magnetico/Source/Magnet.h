#pragma once

#include "MagnetEntity.h"

#include <axmol.h>

#include <vector>

class Magnet : public ax::Node {
public:
	Magnet(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index);
	
	static Magnet* create(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index);

	void updatePositions();
	
private:
	std::vector<MagnetEntity::AttachedEntity>& attachedEntities;
	size_t entityIndex;
};
