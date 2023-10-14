#include "components/systems/MagnetSystem.hpp"

MagnetSystem::MagnetSystem() {
}

msr::airlib::Vector3r MagnetSystem::calculateMagneticFieldAtOrigin(const msr::airlib::Vector3r& origin, const msr::airlib::Vector3r& magnetPosition, MagnetPolarity polarity) {
	
	float distance = sqrt(pow(origin.x() - magnetPosition.x(), 2) + pow(origin.y() - magnetPosition.y(), 2) + pow(origin.z() - magnetPosition.z(), 2));
    
    if (distance < 0.001f)
        distance = 0.001f;
    
    // Calculate the magnetic field strength based on the magnet's distance.
    // The field strength will be inversely proportional to the cube of the distance.
	msr::airlib::Vector3r magneticField = _magneticFieldStrength * (1.0f / (distance * distance * distance)) * (origin - magnetPosition).normalized();
    
    // Considering polarity to determine the direction of the magnetic moment
    if(polarity == MagnetPolarity::SOUTH) {
        magneticField = -magneticField;
    }
    
    return magneticField;
}

msr::airlib::Vector3r MagnetSystem::calculateForceDueToMagnet(const msr::airlib::Vector3r& origin, const msr::airlib::Vector3r& magnetPosition, const msr::airlib::Vector3r& affectedMagnetPosition, MagnetPolarity polarity) {
	msr::airlib::Vector3r field = calculateMagneticFieldAtOrigin(magnetPosition, affectedMagnetPosition, polarity);
    return field;
}

msr::airlib::Vector3r MagnetSystem::combineFieldsOrForces(const msr::airlib::Vector3r& origin) {
	
	// @TODO
	throw std::exception();
	
    return msr::airlib::Vector3r(0, 0, 0);
}


std::shared_ptr<Node> MagnetSystem::attach(float radius, MagnetDirection direction, MagnetPolarity polarity) {
	msr::airlib::Vector3r position(0, 0, 0);
    
    switch (direction) {
        case MagnetDirection::NORTH:
            position =msr::airlib::Vector3r(0, radius, 0);
            break;
        case MagnetDirection::NORTHEAST:
            position = msr::airlib::Vector3r(radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::EAST:
            position = msr::airlib::Vector3r(radius, 0, 0);
            break;
        case MagnetDirection::SOUTHEAST:
            position = msr::airlib::Vector3r(radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::SOUTH:
            position = msr::airlib::Vector3r(0, -radius, 0);
            break;
        case MagnetDirection::SOUTHWEST:
            position = msr::airlib::Vector3r(-radius / std::sqrt(2), -radius / std::sqrt(2), 0);
            break;
        case MagnetDirection::WEST:
            position = msr::airlib::Vector3r(-radius, 0, 0);
            break;
        case MagnetDirection::NORTHWEST:
            position = msr::airlib::Vector3r(-radius / std::sqrt(2), radius / std::sqrt(2), 0);
            break;

        case MagnetDirection::FRONT:
            position = msr::airlib::Vector3r(0, 0, -radius);
            break;
            
        case MagnetDirection::BACK:
            position = msr::airlib::Vector3r(0, 0, radius);
            break;

    }
    
    AttachedEntity coilEntity = {position, polarity};
    _attachedEntities.push_back(coilEntity);
    
    auto magnet = std::make_shared<Magnet>(_attachedEntities, _attachedEntities.size());
        
    magnets.push_back(magnet);
	
	return magnet;
}

void MagnetSystem::update(){
    for(auto magnet : magnets){
        magnet->updatePositions();
    }
}
