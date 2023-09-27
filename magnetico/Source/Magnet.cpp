#include "Magnet.h"

Magnet::Magnet(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index) : attachedEntities(entities), entityIndex(index){
    this->scheduleUpdate();
}

Magnet* Magnet::create(std::vector<MagnetEntity::AttachedEntity>& entities, size_t index) {
    Magnet* magnet = new Magnet(entities, index);
    if (magnet && magnet->init()) {
        magnet->autorelease();
        return magnet;
    }
    delete magnet;
    return nullptr;
}

void Magnet::updatePositions(){
    attachedEntities[entityIndex - 1].position = getWorldPosition3D();
}
