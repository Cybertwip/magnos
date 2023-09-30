#pragma once

#include "MagnetEntity.h"
#include "Magnet.h"

class MagnetSystem : public MagnetEntity {
private:
	static constexpr float mu_0 = 4 * M_PI * 1e-7;  // Vacuum permeability
	//float _magneticFieldStrength = 0.0015f;  // T, approximate value for N52 neodymium bar magnet
	float _magneticFieldStrength = 0.003f;  // T, approximate value for a common ferrite magnet

	std::vector<Magnet*> magnets;

public:

	MagnetSystem();
	~MagnetSystem() override = default;
	
	ax::Vec3 calculateMagneticFieldAtOrigin(ax::Vec3 origin, ax::Vec3 magnetPosition, MagnetPolarity polarity);
	
	ax::Vec3 calculateForceDueToMagnet(const ax::Vec3& origin, const ax::Vec3& magnetPosition, const ax::Vec3& affectedMagnetPosition, MagnetPolarity polarity);
	
	ax::Vec3 combineFieldsOrForces(const ax::Vec3&) override;
	
	void attachToDisk(ax::Node* disk, float radius, MagnetDirection direction, MagnetPolarity polarity) override;
	
	void update();

};
