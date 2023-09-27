#include "Coil.h"

Coil::Coil(std::vector<CoilEntity::AttachedEntity>& entities, size_t index) : attachedEntities(entities), entityIndex(index){
    this->scheduleUpdate();
}

Coil* Coil::create(std::vector<CoilEntity::AttachedEntity>& entities, size_t index) {
    Coil* coil = new Coil(entities, index);
    if (coil && coil->init()) {
        coil->autorelease();
        return coil;
    }
    delete coil;
    return nullptr;
}
void Coil::updatePositions(){
    attachedEntities[entityIndex - 1].position = getWorldPosition3D();
}
