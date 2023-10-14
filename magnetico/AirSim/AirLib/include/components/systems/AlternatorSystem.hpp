#pragma once

#include "components/Coil.hpp"

#include "common/Common.hpp"

class AlternatorSystem : public CoilEntity {
public:
	float emf = 0;
	float current = 0;
	float area = 0;
	float turns = 0;
	
	std::vector<std::shared_ptr<Coil>> coils;

public:
	AlternatorSystem(float coilArea, float coilTurns);

	msr::airlib::Vector3r computeMagneticField(AttachedEntity& coil, const msr::airlib::Vector3r& point, MagnetPolarity polarity) const;
	
	msr::airlib::Vector3r combineFieldsOrForces(const msr::airlib::Vector3r& origin) override;
	
	std::shared_ptr<Node> attach(float radius, MagnetDirection direction, MagnetPolarity polarity);
	void update();

};
