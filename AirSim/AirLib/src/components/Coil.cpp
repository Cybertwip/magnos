#include "components/Coil.hpp"

Coil::Coil(std::vector<CoilEntity::AttachedEntity>& entities, size_t index) : attachedEntities(entities), entityIndex(index){
}

void Coil::updatePositions(){
	
	if (entityIndex > 0 && entityIndex <= attachedEntities.size()) {
		attachedEntities[entityIndex - 1].position = getWorldPosition3D();
	}
}
