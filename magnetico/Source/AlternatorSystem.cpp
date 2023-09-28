#include "AlternatorSystem.h"

AlternatorSystem::AlternatorSystem(float coilArea, float coilTurns)
: area(coilArea), turns(coilTurns){
    
}


ax::Vec3 AlternatorSystem::computeMagneticField(AttachedEntity& coil, const ax::Vec3& point, MagnetPolarity polarity) const {
    return {};
}


ax::Vec3 AlternatorSystem::combineFieldsOrForces() {
    return {};
}

void AlternatorSystem::attachToDisk(ax::Node* node, float radius, MagnetDirection direction, MagnetPolarity polarity) {
    
    ax::Vec3 position(0, 0, 0);
	float rotationAngle = 0.0f; // Initialize rotation angle

	radius = radius + area;
	
    switch (direction) {
        case MagnetDirection::NORTH:
            position = ax::Vec3(0, radius, 0);
            break;
        case MagnetDirection::NORTHEAST:
            position = ax::Vec3(radius / std::sqrt(2), radius / std::sqrt(2), 0);
			rotationAngle = 45.0f; // Rotate 45 degrees
            break;
        case MagnetDirection::EAST:
            position = ax::Vec3(radius, 0, 0);
            break;
        case MagnetDirection::SOUTHEAST:
            position = ax::Vec3(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
			rotationAngle = 45.0f; // Rotate 45 degrees
            break;
        case MagnetDirection::SOUTH:
            position = ax::Vec3(0, -radius, 0);
            break;
        case MagnetDirection::SOUTHWEST:
            position = ax::Vec3(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
			rotationAngle = 45.0f; // Rotate 45 degrees
            break;
        case MagnetDirection::WEST:
            position = ax::Vec3(-radius, 0, 0);
            break;
        case MagnetDirection::NORTHWEST:
            position = ax::Vec3(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
			rotationAngle = 45.0f; // Rotate 45 degrees
            break;
        case MagnetDirection::FRONT:
            position = ax::Vec3(0, 0, -radius);
            break;

        case MagnetDirection::BACK:
            position = ax::Vec3(0, 0, radius);
            break;
    }
    
    _attachedEntities.push_back({position, polarity, area, turns});

    ax::Mesh* magnetMesh = createCube(area);
    auto magnetRenderer = ax::MeshRenderer::create();
    magnetRenderer->addMesh(magnetMesh);
    magnetRenderer->setPosition3D(ax::Vec3(0, 0, 0));
    magnetRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
    magnetRenderer->setTexture("gold.jpg");

	magnetRenderer->setRotation3D(ax::Vec3(0, 0, rotationAngle));
    
    auto coil = Coil::create(_attachedEntities, _attachedEntities.size());
    
    coil->setPosition3D(position);
    coil->addChild(magnetRenderer);
    
    node->addChild(coil);
    
    coils.push_back(coil);

}

void AlternatorSystem::update(){
    for(auto coil : coils){
        coil->updatePositions();
    }
}
