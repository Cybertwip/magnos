#pragma once

#include "Coil.h"

class AlternatorSystem : public CoilEntity {
public:
	float emf = 0;
	float current = 0;
	float area = 0;
	float turns = 0;
	
	std::vector<Coil*> coils;

public:
	AlternatorSystem(float coilArea, float coilTurns);

	ax::Vec3 computeMagneticField(AttachedEntity& coil, const ax::Vec3& point, MagnetPolarity polarity) const;
	
	ax::Vec3 combineFieldsOrForces() override;
	
	void attachToDisk(ax::Node* node, float radius, MagnetDirection direction, MagnetPolarity polarity) override;	
	void update();

};
