#include "MagnetSystem.h"
#include "Utils3d.h"

MagnetSystem::MagnetSystem() {
}

ax::Vec3 MagnetSystem::calculateMagneticFieldAtOrigin(ax::Vec3 origin, ax::Vec3 magnetPosition, MagnetPolarity polarity) {
	ax::Vec3 r = origin - magnetPosition;
	float distance = r.length();
    
    if (distance < 0.001f)
        distance = 0.001f;
    
    // Calculate the magnetic field strength based on the magnet's distance.
    // The field strength will be inversely proportional to the cube of the distance.
    ax::Vec3 magneticField = _magneticFieldStrength * (1.0f / (distance * distance * distance)) * magnetPosition.getNormalized();
    
    // Considering polarity to determine the direction of the magnetic moment
    if(polarity == MagnetPolarity::SOUTH) {
        magneticField = -magneticField;
    }
    
    return magneticField;
}

ax::Vec3 MagnetSystem::calculateForceDueToMagnet(const ax::Vec3& magnetPosition, const ax::Vec3& affectedMagnetPosition, MagnetPolarity polarity) {
    ax::Vec3 field = calculateMagneticFieldAtOrigin(magnetPosition, affectedMagnetPosition, polarity);
    return field;
}

ax::Vec3 MagnetSystem::combineFieldsOrForces(const ax::Vec3& origin) {
    ax::Vec3 totalForce(0, 0, 0);
    
    for (const auto& [magnetPosition, polarity] : _attachedEntities) {
        // Modify force calculation based on polarity if required.
        // For simplicity, assuming polarity does not change the force calculation for now.
        totalForce += calculateMagneticFieldAtOrigin(origin, magnetPosition, polarity);
    }
    
    return totalForce;
}


void MagnetSystem::attachToDisk(ax::Node* disk, float radius, MagnetDirection direction, MagnetPolarity polarity) {
    ax::Vec3 position(0, 0, 0);
    
    switch (direction) {
        case MagnetDirection::NORTH:
            position = ax::Vec3(0, radius, 0);
            break;
        case MagnetDirection::NORTHEAST:
            position = ax::Vec3(radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::EAST:
            position = ax::Vec3(radius, 0, 0);
            break;
        case MagnetDirection::SOUTHEAST:
            position = ax::Vec3(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::SOUTH:
            position = ax::Vec3(0, -radius, 0);
            break;
        case MagnetDirection::SOUTHWEST:
            position = ax::Vec3(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::WEST:
            position = ax::Vec3(-radius, 0, 0);
            break;
        case MagnetDirection::NORTHWEST:
            position = ax::Vec3(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;

        case MagnetDirection::FRONT:
            position = ax::Vec3(0, 0, -radius);
            break;
            
        case MagnetDirection::BACK:
            position = ax::Vec3(0, 0, radius);
            break;

    }
    
    AttachedEntity coilEntity = {position, polarity};
    _attachedEntities.push_back(coilEntity);


    ax::Mesh* magnetMesh = createCube(radius / 24);
    auto magnetRenderer = ax::MeshRenderer::create();
    magnetRenderer->addMesh(magnetMesh);
    magnetRenderer->setPosition3D(ax::Vec3(0, 0, 0));
    magnetRenderer->setMaterial(ax::MeshMaterial::createBuiltInMaterial(ax::MeshMaterial::MaterialType::UNLIT, false));
    magnetRenderer->setTexture("black.jpg");
    
    auto magnet = Magnet::create(_attachedEntities, _attachedEntities.size());
    
    magnet->setPosition3D(position);
    magnet->addChild(magnetRenderer);
    
    disk->addChild(magnet);
    
    magnets.push_back(magnet);
}

void MagnetSystem::update(){
    for(auto magnet : magnets){
        magnet->updatePositions();
    }
}
