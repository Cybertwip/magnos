#include "components/Magnet.hpp"

Magnet::Magnet(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index) : attachedEntities(entities), entityIndex(index){
}

void Magnet::updatePositions(){
	
	if (entityIndex > 0 && entityIndex <= attachedEntities.size()) {
		attachedEntities[entityIndex - 1].position = getWorldPosition3D();
	}
}
