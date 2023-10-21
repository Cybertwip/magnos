#include "components/systems/AlternatorSystem.hpp"

AlternatorSystem::AlternatorSystem(float coilArea, float coilTurns)
: area(coilArea), turns(coilTurns){
    
}


msr::airlib::Vector3r AlternatorSystem::computeMagneticField(AttachedEntity& coil, const msr::airlib::Vector3r& point, MagnetPolarity polarity) const {
    return {};
}


msr::airlib::Vector3r AlternatorSystem::combineFieldsOrForces(const msr::airlib::Vector3r& origin) {
    return {};
}

std::shared_ptr<msr::airlib::Node> AlternatorSystem::attach(float radius, MagnetDirection direction, MagnetPolarity polarity) {
	
	msr::airlib::Vector3r position(0, 0, 0);
	
	switch (direction) {
		case MagnetDirection::NORTH:
			position = msr::airlib::Vector3r(0, radius, 0);
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
	
	
	AttachedEntity coilEntity = {position, polarity, area, turns};
	
	_attachedEntities.push_back(coilEntity);
	
	auto coil = std::make_shared<Coil>(_attachedEntities, _attachedEntities.size());
	
	coil->setPosition3D(position);
	
	coils.push_back(coil);
	
	return coil;
}

void AlternatorSystem::update(){
    for(auto coil : coils){
        coil->updatePositions();
    }
}
